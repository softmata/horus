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

    /// Grow the shared memory region to `new_size` bytes.
    ///
    /// # Safety
    ///
    /// Caller must ensure no concurrent readers/writers to the raw memory.
    /// The caller must also ensure unique access to this ShmRegion (e.g., by
    /// guaranteeing single-thread ownership). This is needed because the method
    /// takes `&self` for Arc compatibility but internally mutates via raw pointer.
    #[allow(invalid_reference_casting)] // Single-thread contract: caller guarantees exclusive access
    pub unsafe fn grow_unchecked(&self, new_size: usize) -> HorusResult<()> {
        // SAFETY: caller guarantees exclusive access. We need &mut for grow_unchecked
        // but have &self (through Arc). The Topic single-thread contract ensures safety.
        let ptr = &self.0 as *const horus_sys::shm::ShmRegion as *mut horus_sys::shm::ShmRegion;
        let inner = &mut *ptr;
        inner
            .grow_unchecked(new_size)
            .map_err(|e| crate::error::HorusError::Memory(e.to_string().into()))
    }
}

// Thread safety — delegates to horus_sys::shm::ShmRegion which is Send + Sync
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shm_create_succeeds() {
        let name = format!("core_shm_test_{}", std::process::id());
        let region = ShmRegion::new(&name, 4096);
        assert!(
            region.is_ok(),
            "ShmRegion::new should succeed: {:?}",
            region.err()
        );
        drop(region);
    }

    #[test]
    fn test_shm_owner_is_first_creator() {
        let name = format!("core_shm_owner_{}", std::process::id());
        let region = ShmRegion::new(&name, 1024).unwrap();
        assert!(region.is_owner(), "first creator should be owner");
        drop(region);
    }

    #[test]
    fn test_shm_len_matches_requested() {
        let name = format!("core_shm_len_{}", std::process::id());
        let region = ShmRegion::new(&name, 8192).unwrap();
        assert_eq!(region.len(), 8192);
        drop(region);
    }

    #[test]
    fn test_shm_write_then_read() {
        let name = format!("core_shm_rw_{}", std::process::id());
        let mut region = ShmRegion::new(&name, 4096).unwrap();
        region.as_slice_mut()[0..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(&region.as_slice()[0..4], &[0xDE, 0xAD, 0xBE, 0xEF]);
        drop(region);
    }

    #[test]
    fn test_shm_as_ptr_not_null() {
        let name = format!("core_shm_ptr_{}", std::process::id());
        let region = ShmRegion::new(&name, 1024).unwrap();
        assert!(!region.as_ptr().is_null());
        drop(region);
    }

    #[test]
    fn test_shm_zero_size_returns_error() {
        let result = ShmRegion::new("core_zero_size", 0);
        assert!(result.is_err(), "size=0 should return HorusError");
    }

    #[test]
    fn test_shm_empty_name_returns_error() {
        let result = ShmRegion::new("", 1024);
        assert!(result.is_err(), "empty name should return HorusError");
    }

    #[test]
    fn test_shm_error_is_memory_variant() {
        let result = ShmRegion::new("", 1024);
        match result {
            Err(crate::error::HorusError::Memory(_)) => {} // correct
            Err(other) => panic!("expected HorusError::Memory, got: {:?}", other),
            Ok(_) => panic!("expected error"),
        }
    }

    #[test]
    fn test_shm_zero_initialized() {
        let name = format!("core_shm_zero_{}", std::process::id());
        let region = ShmRegion::new(&name, 512).unwrap();
        assert!(
            region.as_slice().iter().all(|&b| b == 0),
            "newly created region should be zero-initialized"
        );
        drop(region);
    }
}
