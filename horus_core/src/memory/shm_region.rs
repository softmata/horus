// Thin wrapper around horus_sys::shm::ShmRegion
//
// All platform-specific shared memory code now lives in horus_sys::shm.
// This module re-exports ShmRegion with HorusResult error conversion.

use crate::error::HorusResult;
use std::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};

/// Cross-platform shared memory region for high-performance IPC.
///
/// Delegates to [`horus_sys::shm::ShmRegion`] for the platform-specific mapping.
///
/// # Why the base address is mirrored into atomics
///
/// [`Self::grow`] replaces the mapping, and the replacement lands at a DIFFERENT
/// address.  `Topic` shares one `Arc<ShmRegion>` across every clone, and clones
/// live on different threads, so a grow races every other clone's [`Self::as_ptr`].
/// Reading the inner region directly was wrong twice over:
///
/// 1. It formed a `&horus_sys::shm::ShmRegion` while the growing thread held a
///    `&mut` to that same value — an aliasing violation, i.e. UB, regardless of
///    what the hardware does with the load.
/// 2. The pointer it handed back was the OLD base, which the grow had just
///    `munmap`ed.  That is a use-after-free, and it was *observed*, not theorised:
///    a SIGSEGV reading `TopicHeader::is_verbose` through a stale `header_ptr`
///    (`0x780510d2b000`, while the live mapping was at `0x780510d37000`), and a
///    faulting atomic **store** inside `BackendMigrator::perform_migration` — a
///    wild write that faulted only because the address happened to be unmapped
///    rather than recycled.  Both reproduce in roughly half of the `horus_core`
///    lib-test runs once the box is oversubscribed.
///
/// So `base`, `len` and `owner` are mirrored out of the inner region: readers
/// touch only those and never the `UnsafeCell`, which removes the aliasing
/// hazard.  The horus_sys backends additionally RETAIN the mapping a grow
/// replaces instead of unmapping it, so a pointer some other thread loaded a
/// moment earlier stays mapped — and, being a second `MAP_SHARED` view of the
/// same file, stays *coherent* rather than merely non-faulting.
///
/// Grows are serialized by `grow_lock`, because two producer clones can both
/// reach the auto-grow path.  Growth is `#[cold]` and rare and the per-message
/// paths never call any of this, so the mutex is off the hot path entirely.
#[derive(Debug)]
pub struct ShmRegion {
    inner: std::cell::UnsafeCell<horus_sys::shm::ShmRegion>,
    /// Serializes [`Self::grow`] against itself and against [`Self::as_slice`],
    /// the only two operations that touch `inner`.
    grow_lock: std::sync::Mutex<()>,
    /// Current mapping base, republished after each successful grow.
    base: AtomicPtr<u8>,
    /// Current mapping length, republished with `base`.
    len: AtomicUsize,
    /// Immutable after construction; mirrored so readers never touch `inner`.
    owner: bool,
}

impl ShmRegion {
    /// Create or open a shared memory region.
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        horus_sys::shm::ShmRegion::new(name, size)
            .map(Self::from_inner)
            .map_err(|e| crate::error::HorusError::Memory(e.to_string().into()))
    }

    fn from_inner(region: horus_sys::shm::ShmRegion) -> Self {
        let base = region.as_ptr() as *mut u8;
        let len = region.len();
        let owner = region.is_owner();
        Self {
            inner: std::cell::UnsafeCell::new(region),
            grow_lock: std::sync::Mutex::new(()),
            base: AtomicPtr::new(base),
            len: AtomicUsize::new(len),
            owner,
        }
    }

    /// Raw pointer to the mapped memory.
    ///
    /// Stays valid even if another thread grows the region concurrently: the
    /// replaced mapping is retained rather than unmapped.  It may become *stale*
    /// (addressing the pre-grow layout), which callers resolve by re-deriving
    /// their pointers when they observe the migration epoch bump.
    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        self.base.load(Ordering::Acquire)
    }

    /// View the mapped memory as a byte slice.
    ///
    /// Takes `grow_lock` rather than reading the two atomics, because a slice
    /// needs base and length to describe the SAME mapping: loading them
    /// separately could pair a pre-grow base with a post-grow length and run off
    /// the end of the older, smaller mapping.
    #[inline]
    #[allow(dead_code)]
    pub fn as_slice(&self) -> &[u8] {
        let (ptr, len) = {
            let _guard = self.grow_lock.lock().unwrap_or_else(|e| e.into_inner());
            // SAFETY: `grow_lock` is held, and `grow` is the only writer of
            // `inner`, so no `&mut` to it is live here.
            let inner = unsafe { &*self.inner.get() };
            (inner.as_ptr(), inner.len())
        };
        // SAFETY: the pair was captured atomically w.r.t. grows, and the mapping
        // it describes is never unmapped before `self` is dropped.
        unsafe { std::slice::from_raw_parts(ptr, len) }
    }

    /// View the mapped memory as a mutable byte slice.
    #[inline]
    #[allow(dead_code)]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        self.inner.get_mut().as_slice_mut()
    }

    /// Size of the mapped region in bytes.
    #[inline]
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.len.load(Ordering::Acquire)
    }

    /// Whether this handle is the original creator (responsible for cleanup on drop).
    #[inline]
    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// Grow the underlying SHM region, republishing the new base address.
    ///
    /// Safe to call concurrently with readers and with itself.  This used to be
    /// `unsafe fn grow_unchecked`, carrying a contract ("the caller MUST
    /// guarantee exclusive access") that nothing enforced and that the topic
    /// call sites did not honour — a known limitation recorded in this file that
    /// turned out to be a live use-after-free.  The contract is gone because the
    /// hazards it was standing in for are gone: grows are serialized here, the
    /// base is published atomically, and the replaced mapping is retained.
    ///
    /// What callers still owe is a *correctness* obligation, not a safety one:
    /// after a grow the geometry has changed, so any cached slot pointers must
    /// be re-derived (`point_at_slots`) before the next access.
    ///
    /// A concurrent grow that already reached `new_size` or beyond is treated as
    /// success — the caller's requirement is "at least this big", and the
    /// backend rejects a non-increasing resize.
    pub fn grow(&self, new_size: usize) -> HorusResult<()> {
        let _guard = self.grow_lock.lock().unwrap_or_else(|e| e.into_inner());

        // Someone else may have grown past this while we waited for the lock.
        let current = self.len.load(Ordering::Relaxed);
        if current >= new_size {
            return Ok(());
        }

        // Grow at least geometrically, even when the caller asked for a size
        // only slightly larger. `handle_epoch_change` derives its request from a
        // PEER process's header fields, which are under no obligation to move in
        // powers of two: a peer that raises `slot_size` by one per epoch would
        // otherwise drive one retained mapping per epoch. Doubling collapses that
        // chain into the `current >= new_size` early return above, which is what
        // keeps `MAX_RETIRED_MAPPINGS` a generous bound rather than a limit real
        // workloads approach. Over-allocating costs address space and a lazily
        // extended tmpfs file, not resident memory — `make_resident` caps what it
        // faults in at `residency_policy().max_bytes` regardless of the mapping.
        let new_size = new_size.max(current.saturating_mul(2));

        // SAFETY: `grow_lock` is held, so this is the only live reference to
        // `inner`; `as_slice` is the only other reader of it and takes the same
        // lock.  The horus_sys contract ("no other thread touching the region")
        // is met for the region's *fields*; its mapped *bytes* stay valid for
        // other threads because the old mapping is retained, not unmapped.
        let result = unsafe { (*self.inner.get()).grow_unchecked(new_size) };
        result.map_err(|e| crate::error::HorusError::Memory(e.to_string().into()))?;

        // Publish base and length only after the grow has succeeded, so a
        // failed grow leaves the previous mapping described correctly.
        // SAFETY: as above.
        let inner = unsafe { &*self.inner.get() };
        self.base
            .store(inner.as_ptr() as *mut u8, Ordering::Release);
        self.len.store(inner.len(), Ordering::Release);
        Ok(())
    }
}

// Thread safety.
//
// `UnsafeCell` is `!Sync`, so this impl is load-bearing.  The invariant it
// asserts: `inner` is touched only by `grow` and `as_slice`, which serialize on
// `grow_lock`, and by `as_slice_mut`/`Drop`, which have `&mut self`.  Every
// other accessor reads the mirrored `base`/`len`/`owner` instead.  Sharing an
// `Arc<ShmRegion>` across threads is therefore sound without a caller-side
// contract, which is the point: the previous contract was unenforced and
// unhonoured.
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

    /// A pointer taken before a grow must still be readable afterwards — and
    /// must still read the *same bytes* as the new mapping.
    ///
    /// This is the property the whole fix rests on. The crash was a sibling
    /// clone dereferencing exactly such a pointer after the grow had unmapped
    /// it. Retention alone would only make the read non-faulting; what makes it
    /// *correct* is that both mappings are `MAP_SHARED` views of the same file
    /// at offset 0, so the overlapping prefix is the same page-cache pages.
    #[test]
    fn a_pointer_taken_before_a_grow_stays_mapped_and_coherent() {
        let name = format!("core_shm_grow_retain_{}", std::process::id());
        let region = ShmRegion::new(&name, 4096).unwrap();
        let old_ptr = region.as_ptr();

        // SAFETY: `old_ptr` is the base of a live 4096-byte mapping.
        unsafe { (old_ptr as *mut u8).write(0xAB) };

        region.grow(64 * 1024).unwrap();
        let new_ptr = region.as_ptr();
        assert_ne!(
            old_ptr, new_ptr,
            "the premise of this test is that growing MOVES the mapping; if that \
             stops being true the retention machinery is guarding nothing"
        );

        // Non-faulting: this is the dereference that used to SIGSEGV.
        // SAFETY: the old mapping is retained for the region's lifetime.
        assert_eq!(
            unsafe { *old_ptr },
            0xAB,
            "the retained mapping was unmapped"
        );

        // Coherent, not merely mapped: a write through the NEW base is visible
        // through the OLD one, because they are the same physical pages.
        // SAFETY: both pointers are bases of live mappings of the same file.
        unsafe { (new_ptr as *mut u8).write(0xCD) };
        assert_eq!(
            unsafe { *old_ptr },
            0xCD,
            "old and new mappings are not views of the same pages, so a stale \
             reader would see a stale snapshot rather than live data"
        );
    }

    /// Retention is bounded, and exceeding the bound fails the grow rather than
    /// unmapping to make room.
    #[test]
    fn retained_mappings_are_capped_rather_than_unmapped() {
        let name = format!("core_shm_grow_cap_{}", std::process::id());
        let region = ShmRegion::new(&name, 4096).unwrap();
        let first = region.as_ptr();

        // Each call asks for one byte more than the last. The doubling floor in
        // `grow` turns all but the first few into no-ops, so the cap is not
        // reached — which is the point of the floor.
        for i in 0..64usize {
            let _ = region.grow(4097 + i);
        }

        // SAFETY: retained for the region's lifetime; that is what is asserted.
        assert_eq!(
            unsafe { *first },
            unsafe { *region.as_ptr() },
            "every mapping this region ever handed out must stay readable"
        );
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
