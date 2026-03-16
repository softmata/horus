// Thin wrapper around horus_sys::shm::ShmRegion
//
// All platform-specific shared memory code now lives in horus_sys::shm.
// This module re-exports ShmRegion with HorusResult error conversion.

use crate::error::HorusResult;

/// Cross-platform shared memory region for high-performance IPC.
///
/// Delegates to [`horus_sys::shm::ShmRegion`] for platform-specific implementation.
/// This wrapper converts `anyhow::Error` to `HorusError::Memory` for compatibility
/// with the horus_core error hierarchy.
#[derive(Debug)]
pub struct ShmRegion(horus_sys::shm::ShmRegion);

impl ShmRegion {
    /// Create or open a shared memory region.
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        horus_sys::shm::ShmRegion::new(name, size)
            .map(Self)
            .map_err(|e| crate::error::HorusError::Memory(e.to_string().into()))
    }

    /// Raw pointer to the mapped memory.
    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        self.0.as_ptr()
    }

    /// View the mapped memory as a byte slice.
    #[inline]
    #[allow(dead_code)]
    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }

    /// View the mapped memory as a mutable byte slice.
    #[inline]
    #[allow(dead_code)]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        self.0.as_slice_mut()
    }

    /// Size of the mapped region in bytes.
    #[inline]
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether this handle is the original creator (responsible for cleanup on drop).
    #[inline]
    pub fn is_owner(&self) -> bool {
        self.0.is_owner()
    }
}

// Thread safety — delegates to horus_sys::shm::ShmRegion which is Send + Sync
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}
