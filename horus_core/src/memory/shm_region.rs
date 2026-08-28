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
///
/// The inner region is wrapped in an `UnsafeCell` because [`Self::grow_unchecked`]
/// mutates it through a shared reference.  It used to be a plain field that
/// `grow_unchecked` cast from `&T` to `&mut T` — undefined behaviour regardless
/// of how exclusive the caller's access is, and silenced with
/// `#[allow(invalid_reference_casting)]`.  `horus_sys::shm::ShmRegion` holds no
/// `UnsafeCell` of its own, so it is `Freeze`: rustc lowers `&self` with
/// `noalias readonly` and is entitled to cache `self.0.mmap` / `self.0.size`
/// across the call.  `grow_unchecked` replaces both (dropping the old `MmapMut`,
/// i.e. `munmap`), so a cached pointer is a dangling one — a miscompilation
/// hazard that comes and goes with inlining.  `UnsafeCell` is the only sound
/// way to mutate behind `&self`, and it also removes the `Freeze` assumption.
#[derive(Debug)]
pub struct ShmRegion(std::cell::UnsafeCell<horus_sys::shm::ShmRegion>);

impl ShmRegion {
    /// Create or open a shared memory region.
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        horus_sys::shm::ShmRegion::new(name, size)
            .map(|region| Self(std::cell::UnsafeCell::new(region)))
            .map_err(|e| crate::error::HorusError::Memory(e.to_string().into()))
    }

    /// Raw pointer to the mapped memory.
    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        // SAFETY: see the `Sync` impl below — readers and `grow_unchecked` are
        // serialized by the caller's exclusive-access contract.
        unsafe { (*self.0.get()).as_ptr() }
    }

    /// View the mapped memory as a byte slice.
    #[inline]
    #[allow(dead_code)]
    pub fn as_slice(&self) -> &[u8] {
        // SAFETY: as for `as_ptr`.
        unsafe { (*self.0.get()).as_slice() }
    }

    /// View the mapped memory as a mutable byte slice.
    #[inline]
    #[allow(dead_code)]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        self.0.get_mut().as_slice_mut()
    }

    /// Size of the mapped region in bytes.
    #[inline]
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        // SAFETY: as for `as_ptr`.
        unsafe { (*self.0.get()).len() }
    }

    /// Whether this handle is the original creator (responsible for cleanup on drop).
    #[inline]
    pub fn is_owner(&self) -> bool {
        // SAFETY: as for `as_ptr`.
        unsafe { (*self.0.get()).is_owner() }
    }

    /// Grow the underlying SHM region without reallocating.
    ///
    /// # Safety
    ///
    /// Caller MUST guarantee exclusive access to this ShmRegion.
    /// The Topic single-thread-per-instance contract provides this guarantee:
    /// each Topic<T> instance is only accessed from one thread, and grow is
    /// only called during migration which holds the migration lock.
    ///
    /// Violating this invariant is undefined behavior (data race on mmap).
    ///
    /// **Known limitation** — the contract above is *not* enforced anywhere.
    /// This call replaces the mapping, so every concurrent reader's
    /// [`as_ptr`](Self::as_ptr) / [`len`](Self::len) result is invalidated.
    /// Callers must hold the topic header's migration lock across both the grow
    /// and the re-derivation of any cached pointers; the current callers in
    /// `communication/topic` do not, which is tracked separately.
    pub unsafe fn grow_unchecked(&self, new_size: usize) -> HorusResult<()> {
        // SAFETY: Caller guarantees exclusive access per the contract above.
        (*self.0.get())
            .grow_unchecked(new_size)
            .map_err(|e| crate::error::HorusError::Memory(e.to_string().into()))
    }
}

// Thread safety.
//
// `UnsafeCell` is `!Sync`, so this impl is load-bearing rather than the
// redundant delegation its old comment claimed ("delegates to
// horus_sys::shm::ShmRegion which is Send + Sync").  The real invariant it
// asserts: every method except `grow_unchecked` only *reads* the inner region,
// and `grow_unchecked` is `unsafe` precisely because the caller must guarantee
// no other thread is touching this region while it runs.  Sharing an
// `Arc<ShmRegion>` across threads is sound only under that contract.
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
