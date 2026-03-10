//! RAII handle for tensor memory management
//!
//! This module provides a safe wrapper around `Tensor` that automatically
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
//! let handle = TensorHandle::alloc(pool.clone(), &[100, 100], TensorDtype::F32, Device::cpu())?;
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

use super::tensor_pool::{Device, Tensor, TensorDtype, TensorPool};
use crate::error::{HorusError, HorusResult};
use crate::types::MAX_TENSOR_DIMS;
use std::sync::Arc;

/// RAII handle that automatically manages tensor reference counting
///
/// When a `TensorHandle` is cloned, the reference count is incremented.
/// When it is dropped, the reference count is decremented. When the count
/// reaches zero, the tensor's memory slot is returned to the pool.
pub struct TensorHandle {
    tensor: Tensor,
    pool: Arc<TensorPool>,
}

impl TensorHandle {
    /// Create a new handle from an existing tensor descriptor
    ///
    /// This increments the reference count. Use this when receiving a tensor
    /// from Topic or when wrapping a tensor you don't own yet.
    pub fn new(tensor: Tensor, pool: Arc<TensorPool>) -> Self {
        pool.retain(&tensor);
        Self { tensor, pool }
    }

    /// Create a handle without incrementing the reference count.
    ///
    /// Use this when you've just allocated a tensor and already have a refcount of 1.
    /// This is an internal API — prefer [`alloc`] for new tensors.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::InvalidDescriptor`] if `tensor.pool_id` does not
    /// match `pool.pool_id()`.  A mismatch causes `clone()` to call `retain()`
    /// on the wrong pool, silently corrupting refcounts and risking use-after-free.
    ///
    /// In debug builds the mismatch causes an immediate panic (via
    /// `debug_assert!`) before reaching the `Err` return.
    pub(crate) fn from_owned(tensor: Tensor, pool: Arc<TensorPool>) -> HorusResult<Self> {
        debug_assert_eq!(
            tensor.pool_id,
            pool.pool_id(),
            "TensorHandle::from_owned: tensor.pool_id ({}) does not match pool.pool_id ({}); \
             clone() would corrupt refcounts on the wrong pool",
            tensor.pool_id,
            pool.pool_id()
        );
        if tensor.pool_id != pool.pool_id() {
            return Err(HorusError::InvalidDescriptor(format!(
                "TensorHandle::from_owned: tensor.pool_id ({}) does not match pool.pool_id ({})",
                tensor.pool_id,
                pool.pool_id()
            )));
        }
        Ok(Self { tensor, pool })
    }

    /// Allocate a new tensor and return a handle
    ///
    /// This is the recommended way to create new tensors.
    pub fn alloc(
        pool: Arc<TensorPool>,
        shape: &[u64],
        dtype: TensorDtype,
        device: Device,
    ) -> HorusResult<Self> {
        let tensor = pool.alloc(shape, dtype, device)?;
        // SAFETY: tensor was just allocated from `pool`, so pool_ids always match.
        Ok(Self::from_owned(tensor, pool)
            .expect("tensor just allocated from pool always has matching pool_id"))
    }

    /// Get the underlying tensor descriptor
    ///
    /// The descriptor can be sent through Topic. The receiver should
    /// wrap it in a new `TensorHandle` to manage the reference count.
    #[inline]
    pub fn tensor(&self) -> &Tensor {
        &self.tensor
    }

    /// Get mutable reference to the underlying tensor descriptor
    ///
    /// Use with care - modifying the tensor descriptor can break invariants.
    #[inline]
    pub fn tensor_mut(&mut self) -> &mut Tensor {
        &mut self.tensor
    }

    /// Get raw pointer to tensor data
    #[inline]
    pub fn data_ptr(&self) -> *mut u8 {
        self.pool.data_ptr(&self.tensor)
    }

    /// Get tensor data as a byte slice.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::Memory`] if the tensor descriptor is invalid (pool_id
    /// mismatch or out-of-bounds offset/size).  A properly constructed `TensorHandle`
    /// should never produce this error during normal operation.
    #[inline]
    pub fn data_slice(&self) -> crate::error::HorusResult<&[u8]> {
        self.pool.data_slice(&self.tensor)
    }

    /// Get tensor data as a mutable byte slice.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::Memory`] if the tensor descriptor is invalid.
    #[inline]
    pub fn data_slice_mut(&self) -> crate::error::HorusResult<&mut [u8]> {
        self.pool.data_slice_mut(&self.tensor)
    }

    /// Get tensor data as a typed slice.
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::Memory`] if the tensor descriptor is invalid.
    #[inline]
    pub unsafe fn data_as<T: Copy>(&self) -> crate::error::HorusResult<&[T]> {
        let bytes = self.data_slice()?;
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
        Ok(std::slice::from_raw_parts(ptr, len))
    }

    /// Get tensor data as a mutable typed slice.
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    ///
    /// # Errors
    ///
    /// Returns [`HorusError::Memory`] if the tensor descriptor is invalid.
    #[inline]
    #[allow(clippy::mut_from_ref)] // mmap'd shared memory: mutability is OS-level, not Rust borrow-level
    pub unsafe fn data_as_mut<T: Copy>(&self) -> crate::error::HorusResult<&mut [T]> {
        let bytes = self.data_slice_mut()?;
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
        Ok(std::slice::from_raw_parts_mut(ptr, len))
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
    pub fn device(&self) -> Device {
        self.tensor.device()
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
        self.tensor.is_cpu()
    }

    /// Check if tensor is on CUDA
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.tensor.is_cuda()
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
            allocator: Default::default(),
        };
        Arc::new(TensorPool::new(pool_id, config).expect("Failed to create pool"))
    }

    #[test]
    fn test_handle_alloc() {
        let pool = create_test_pool();
        let handle = TensorHandle::alloc(pool.clone(), &[10, 20], TensorDtype::F32, Device::cpu())
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
        let handle1 = TensorHandle::alloc(pool.clone(), &[10], TensorDtype::U8, Device::cpu())
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
        let handle = TensorHandle::alloc(pool.clone(), &[4], TensorDtype::F32, Device::cpu())
            .expect("Failed to allocate");

        // SAFETY: Handle was allocated with shape [4] and dtype F32, so data_as_mut::<f32>()
        // returns a valid &mut [f32; 4] slice backed by the tensor's shared memory region.
        let data = unsafe { handle.data_as_mut::<f32>().unwrap() };
        data[0] = 1.0;
        data[1] = 2.0;
        data[2] = 3.0;
        data[3] = 4.0;

        // SAFETY: Same allocation as above; data_as::<f32>() reads the previously written values.
        let data = unsafe { handle.data_as::<f32>().unwrap() };
        assert_eq!(data, &[1.0, 2.0, 3.0, 4.0]);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    #[test]
    fn test_slice() {
        let pool = create_test_pool();
        let handle = TensorHandle::alloc(pool.clone(), &[10, 5], TensorDtype::F32, Device::cpu())
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

    // ── from_owned pool-identity validation tests ──────────────────────────

    /// from_owned with matching pool_id must succeed.
    #[test]
    fn test_from_owned_matching_pool_succeeds() {
        let pool = create_test_pool();
        let tensor = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        // This is the canonical use: tensor came from pool, IDs match.
        let result = TensorHandle::from_owned(tensor, Arc::clone(&pool));
        assert!(
            result.is_ok(),
            "from_owned with matching pool_id must succeed"
        );
        let handle = result.unwrap();
        assert_eq!(handle.refcount(), 1);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// from_owned with a mismatched pool_id must return Err (release) or panic (debug).
    ///
    /// In release builds the function returns `HorusError::InvalidDescriptor`.
    /// In debug builds the `debug_assert_eq!` fires first, causing a panic —
    /// so we only assert the `Err` path in `#[cfg(not(debug_assertions))]`.
    #[test]
    #[cfg(not(debug_assertions))]
    fn test_from_owned_mismatched_pool_returns_err() {
        let pool_a = create_test_pool();
        let pool_b = create_test_pool();

        // Allocate from pool_a but hand the descriptor to pool_b.
        let tensor = pool_a
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        // pool_a and pool_b have different pool_ids.
        assert_ne!(pool_a.pool_id(), pool_b.pool_id());

        let result = TensorHandle::from_owned(tensor, Arc::clone(&pool_b));
        assert!(
            result.is_err(),
            "from_owned with mismatched pool_id must return Err"
        );
        assert!(
            matches!(
                result.unwrap_err(),
                crate::error::HorusError::InvalidDescriptor(_)
            ),
            "error must be InvalidDescriptor"
        );

        // Release the slot from the correct pool to avoid a leak.
        pool_a.release(&tensor);

        std::fs::remove_file(pool_a.shm_path()).ok();
        std::fs::remove_file(pool_b.shm_path()).ok();
    }

    /// Debug-mode panic test: from_owned with mismatched pool panics via debug_assert.
    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "does not match pool.pool_id")]
    fn test_from_owned_mismatched_pool_panics_in_debug() {
        let pool_a = create_test_pool();
        let pool_b = create_test_pool();

        let tensor = pool_a
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");

        assert_ne!(pool_a.pool_id(), pool_b.pool_id());

        // This must panic (debug_assert fires before the Err return).
        let _ = TensorHandle::from_owned(tensor, Arc::clone(&pool_b));
    }

    // ── Handle lifecycle tests ──────────────────────────────────────────

    /// Dropping a TensorHandle returns the slot to the pool for reuse.
    #[test]
    fn test_handle_drop_returns_slot_to_pool() {
        let pool = create_test_pool();

        let handle =
            TensorHandle::alloc(pool.clone(), &[16], TensorDtype::U8, Device::cpu()).unwrap();
        let slot_id = handle.tensor().slot_id;
        assert_eq!(pool.refcount(handle.tensor()), 1);

        let stats_before = pool.stats();
        assert_eq!(stats_before.allocated_slots, 1);

        drop(handle); // Should free the slot

        let stats_after = pool.stats();
        assert_eq!(
            stats_after.allocated_slots, 0,
            "Slot should be freed after handle drop"
        );

        // Slot should be reusable — allocate again
        let handle2 =
            TensorHandle::alloc(pool.clone(), &[16], TensorDtype::U8, Device::cpu()).unwrap();
        assert_eq!(
            handle2.tensor().slot_id,
            slot_id,
            "Pool should reuse the freed slot"
        );

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// Multiple clones increment refcount correctly and last drop frees the slot.
    #[test]
    fn test_handle_multiple_clones_refcount() {
        let pool = create_test_pool();

        let h1 = TensorHandle::alloc(pool.clone(), &[8], TensorDtype::F32, Device::cpu()).unwrap();
        assert_eq!(h1.refcount(), 1);

        let h2 = h1.clone();
        assert_eq!(h1.refcount(), 2);

        let h3 = h2.clone();
        assert_eq!(h1.refcount(), 3);

        let h4 = h3.clone();
        assert_eq!(h1.refcount(), 4);

        drop(h2);
        assert_eq!(h1.refcount(), 3);

        drop(h4);
        assert_eq!(h1.refcount(), 2);

        drop(h3);
        assert_eq!(h1.refcount(), 1);

        // Stats should still show allocated
        assert_eq!(pool.stats().allocated_slots, 1);

        drop(h1); // Last handle — slot freed
        assert_eq!(pool.stats().allocated_slots, 0);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// Clone survives original being dropped — no use-after-free.
    #[test]
    fn test_handle_clone_survives_original_drop() {
        let pool = create_test_pool();

        let original =
            TensorHandle::alloc(pool.clone(), &[4], TensorDtype::F32, Device::cpu()).unwrap();

        // Write data through original
        // SAFETY: F32 dtype matches f32 type.
        unsafe {
            let data = original.data_as_mut::<f32>().unwrap();
            data[0] = 1.0;
            data[1] = 2.0;
            data[2] = 3.0;
            data[3] = 4.0;
        }

        let clone = original.clone();
        drop(original); // Drop original — clone should still be valid

        // Data should still be accessible through the clone
        // SAFETY: The tensor was written with f32 values above; dtype matches.
        let data = unsafe { clone.data_as::<f32>().unwrap() };
        assert_eq!(data, &[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(clone.refcount(), 1);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// TensorHandle::new() increments refcount (wrapping an existing descriptor).
    #[test]
    fn test_handle_new_increments_refcount() {
        let pool = create_test_pool();

        let tensor = pool
            .alloc(&[8], TensorDtype::U8, Device::cpu())
            .expect("alloc failed");
        assert_eq!(pool.refcount(&tensor), 1);

        // TensorHandle::new() should retain (increment refcount)
        let handle = TensorHandle::new(tensor, Arc::clone(&pool));
        assert_eq!(handle.refcount(), 2); // 1 from alloc + 1 from new()

        drop(handle); // Drop decrements back to 1
        assert_eq!(pool.refcount(&tensor), 1);

        pool.release(&tensor); // Clean up the original alloc
        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// slice_first_dim creates a handle that shares refcount with original.
    #[test]
    fn test_handle_slice_refcount() {
        let pool = create_test_pool();

        let handle =
            TensorHandle::alloc(pool.clone(), &[20, 10], TensorDtype::F32, Device::cpu()).unwrap();
        assert_eq!(handle.refcount(), 1);

        let slice = handle.slice_first_dim(5, 15).expect("slice should work");
        assert_eq!(handle.refcount(), 2);
        assert_eq!(slice.shape(), &[10, 10]);

        // Clone the slice
        let slice2 = slice.clone();
        assert_eq!(handle.refcount(), 3);

        drop(slice);
        assert_eq!(handle.refcount(), 2);

        drop(slice2);
        assert_eq!(handle.refcount(), 1);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// view() creates a handle that shares refcount with original.
    #[test]
    fn test_handle_view_refcount() {
        let pool = create_test_pool();

        let handle =
            TensorHandle::alloc(pool.clone(), &[100], TensorDtype::F32, Device::cpu()).unwrap();
        assert_eq!(handle.refcount(), 1);

        let view = handle.view(&[10, 10]).expect("view should work");
        assert_eq!(handle.refcount(), 2);
        assert_eq!(view.shape(), &[10, 10]);
        assert_eq!(view.numel(), 100);

        drop(view);
        assert_eq!(handle.refcount(), 1);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// Cross-thread clone and drop correctness.
    #[test]
    fn test_handle_cross_thread_clone_drop() {
        use std::thread;

        let pool = create_test_pool();

        let handle =
            TensorHandle::alloc(pool.clone(), &[32], TensorDtype::U8, Device::cpu()).unwrap();

        // Write a pattern
        handle.data_slice_mut().unwrap().fill(0xAB);

        // Clone into 4 threads
        let handles: Vec<_> = (0..4).map(|_| handle.clone()).collect();
        assert_eq!(handle.refcount(), 5); // 1 original + 4 clones

        let threads: Vec<_> = handles
            .into_iter()
            .map(|h| {
                thread::spawn(move || {
                    // Each thread reads the data and verifies it
                    let data = h.data_slice().unwrap();
                    assert!(data.iter().all(|&b| b == 0xAB));
                    assert!(h.refcount() >= 1);
                    // Handle dropped at end of closure
                })
            })
            .collect();

        for t in threads {
            t.join().unwrap();
        }

        assert_eq!(handle.refcount(), 1); // Only original remains

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// Debug format includes useful information.
    #[test]
    fn test_handle_debug_format() {
        let pool = create_test_pool();

        let handle =
            TensorHandle::alloc(pool.clone(), &[3, 4], TensorDtype::F32, Device::cpu()).unwrap();
        let debug = format!("{:?}", handle);

        assert!(debug.contains("TensorHandle"));
        assert!(debug.contains("shape"));
        assert!(debug.contains("dtype"));
        assert!(debug.contains("refcount"));

        std::fs::remove_file(pool.shm_path()).ok();
    }

    /// Handle accessors return correct values.
    #[test]
    fn test_handle_accessors() {
        let pool = create_test_pool();

        let handle =
            TensorHandle::alloc(pool.clone(), &[5, 10], TensorDtype::F64, Device::cpu()).unwrap();

        assert_eq!(handle.shape(), &[5, 10]);
        assert_eq!(handle.dtype(), TensorDtype::F64);
        assert_eq!(handle.numel(), 50);
        assert_eq!(handle.nbytes(), 400); // 50 * 8 bytes
        assert!(handle.is_cpu());
        assert!(!handle.is_cuda());
        assert!(handle.is_contiguous());
        assert!(std::ptr::eq(handle.pool().as_ref(), pool.as_ref()));

        std::fs::remove_file(pool.shm_path()).ok();
    }
}
