//! Auto-managed tensor pools for `Topic<HorusTensor>`
//!
//! Provides convenience methods on `Topic<HorusTensor>` that automatically
//! manage a shared-memory `TensorPool` per topic. Users call `alloc()`,
//! `send_handle()`, and `recv_handle()` instead of managing pools manually.
//!
//! # How it works
//!
//! Each topic name maps to a single `TensorPool` backed by shared memory.
//! The pool is created lazily on first use and shared across all
//! `Topic<HorusTensor>` instances with the same name — even across processes.
//!
//! When sending, only the 232-byte `HorusTensor` descriptor flows through the
//! ring buffer. The actual tensor data stays in the pool — true zero-copy.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::Topic;
//! use horus_types::{HorusTensor, TensorDtype, Device};
//!
//! let topic: Topic<HorusTensor> = Topic::new("camera/rgb")?;
//!
//! // Allocate a 1080p RGB image from the topic's pool
//! let mut handle = topic.alloc(&[1080, 1920, 3], TensorDtype::U8, Device::cpu())?;
//!
//! // Write pixel data
//! let pixels = handle.data_slice_mut();
//! // ... fill pixels ...
//!
//! // Send — only the descriptor is copied, data stays in shared memory
//! topic.send_handle(&handle)?;
//! ```

use std::sync::Arc;

use super::pool_registry::get_or_create_pool;
use super::Topic;
use crate::error::HorusResult;
use crate::memory::{TensorHandle, TensorPool};
use horus_types::{Device, HorusTensor, TensorDtype};

impl Topic<HorusTensor> {
    /// Get or create the auto-managed tensor pool for this topic.
    ///
    /// On first call, opens an existing pool (if another process created it)
    /// or creates a new one backed by shared memory. Subsequent calls return
    /// the cached pool. The pool is shared across all `Topic<HorusTensor>`
    /// instances with the same name within this process.
    pub fn pool(&self) -> Arc<TensorPool> {
        get_or_create_pool(self.name())
    }

    /// Allocate a tensor from this topic's auto-managed pool.
    ///
    /// Returns a `TensorHandle` with RAII refcounting — the tensor slot is
    /// automatically released back to the pool when the handle is dropped.
    pub fn alloc_tensor(
        &self,
        shape: &[u64],
        dtype: TensorDtype,
        device: Device,
    ) -> HorusResult<TensorHandle> {
        let pool = self.pool();
        TensorHandle::alloc(pool, shape, dtype, device)
    }

    /// Send a tensor handle through this topic (zero-copy).
    ///
    /// Increments the tensor's refcount so it stays alive for the receiver,
    /// then sends the 232-byte descriptor through the ring buffer. The actual
    /// tensor data remains in shared memory — no copy.
    pub fn send_handle(&self, handle: &TensorHandle) {
        // Bump refcount so the data survives until the receiver drops its handle
        handle.pool().retain(handle.tensor());
        // Send the lightweight descriptor through the ring
        self.send(*handle.tensor());
    }

    /// Receive a tensor and wrap it in a `TensorHandle` for safe access.
    ///
    /// The returned handle manages the refcount automatically — when dropped,
    /// the tensor's refcount is decremented. Returns `None` if no message
    /// is available.
    pub fn recv_handle(&self) -> Option<TensorHandle> {
        let tensor = self.recv()?;
        let pool = self.pool();
        // from_owned: the sender already incremented the refcount for us
        Some(TensorHandle::from_owned(tensor, pool))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic_tensor_pool_roundtrip() {
        let topic: Topic<HorusTensor> = Topic::new("test/tensor_ext_roundtrip").unwrap();

        // Allocate a tensor
        let handle = topic
            .alloc_tensor(&[2, 3], TensorDtype::F32, Device::cpu())
            .unwrap();

        // Write data
        let data = handle.data_slice_mut();
        let floats: &mut [f32] =
            bytemuck::cast_slice_mut(&mut data[..6 * std::mem::size_of::<f32>()]);
        for (i, v) in floats.iter_mut().enumerate() {
            *v = i as f32;
        }

        // Send
        topic.send_handle(&handle);

        // Receive
        let recv_handle = topic.recv_handle().expect("should receive tensor");
        assert_eq!(recv_handle.shape(), &[2, 3]);
        assert_eq!(recv_handle.dtype(), TensorDtype::F32);

        let recv_data = recv_handle.data_slice();
        let recv_floats: &[f32] =
            bytemuck::cast_slice(&recv_data[..6 * std::mem::size_of::<f32>()]);
        for (i, v) in recv_floats.iter().enumerate() {
            assert_eq!(*v, i as f32);
        }
    }

    #[test]
    fn test_pool_shared_across_calls() {
        let topic: Topic<HorusTensor> = Topic::new("test/tensor_ext_shared_pool").unwrap();

        let pool1 = topic.pool();
        let pool2 = topic.pool();

        // Same Arc (same pool_id)
        assert_eq!(pool1.pool_id(), pool2.pool_id());
    }
}
