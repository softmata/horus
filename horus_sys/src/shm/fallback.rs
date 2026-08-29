// Fallback shared memory for other Unix-like platforms (BSD, etc.)
//
// Uses file-based mmap in /tmp (similar to Linux but without /dev/shm).

use anyhow::{Context, Result};
use memmap2::{MmapMut, MmapOptions};
use std::fs::{File, OpenOptions};
use std::os::unix::io::AsRawFd;
use std::path::PathBuf;

/// Cross-platform shared memory region for high-performance IPC.
///
/// Fallback backend: file-based mmap in `/tmp`.
#[derive(Debug)]
pub struct ShmRegion {
    mmap: MmapMut,
    /// Mappings that `grow_unchecked` replaced, deliberately kept alive.
    /// See the Linux backend's `retired` field for the full rationale: dropping
    /// these `munmap`s address space that other threads still hold pointers
    /// into, because `Topic` shares one `Arc<ShmRegion>` across clones.
    retired: Vec<MmapMut>,
    _file: File,
    path: PathBuf,
    size: usize,
    owner: bool,
}

impl ShmRegion {
    /// Open an existing region without creating it.
    pub fn open_existing(name: &str, minimum_size: usize) -> Result<Self> {
        super::validate_region_name(name)?;
        let path = PathBuf::from("/tmp/horus/topics").join(format!("horus_{}", name));
        let file = OpenOptions::new().read(true).write(true).open(&path)?;
        let size = file.metadata()?.len() as usize;
        anyhow::ensure!(size >= minimum_size, "existing SHM region is too small");
        // Take the same shared lock `new()` does. The module contract is that
        // *every* holder keeps LOCK_SH for the lifetime of its region — that is
        // what makes the last-one-out check in `drop` correct. A holder that
        // joined here without the lock was invisible to it.
        //
        // SAFETY: file.as_raw_fd() is a valid open fd; LOCK_SH is a valid flock op.
        if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) } != 0 {
            anyhow::bail!(
                "Failed to acquire shared lock on SHM file '{}': {}",
                path.display(),
                std::io::Error::last_os_error()
            );
        }
        let mmap = unsafe { MmapOptions::new().len(size).map_mut(&file)? };

        // Pay the page faults at attach instead of inside the first receive
        // loop; see `horus_sys::shm::make_resident` for the policy and its
        // opt-outs.
        // SAFETY: `mmap` owns a live mapping of `size` bytes and outlives the
        // call; `make_resident` neither reads nor writes the region.
        unsafe { super::make_resident(mmap.as_ptr(), size) };

        Ok(Self {
            mmap,
            retired: Vec::new(),
            _file: file,
            path,
            size,
            owner: false,
        })
    }

    /// Create or open a shared memory region.
    pub fn new(name: &str, size: usize) -> Result<Self> {
        super::validate_region_name(name)?;
        // NOTE: this backend is only compiled for platforms that are neither
        // Linux, macOS nor Windows. It stores regions in a fixed, world-writable
        // /tmp path with no namespace and opens them via exists()-then-open,
        // which is a symlink/TOCTOU hazard. It is not hardened here because it
        // is unreachable on every supported target; see the audit notes.
        let horus_shm_dir = PathBuf::from("/tmp/horus/topics");
        std::fs::create_dir_all(&horus_shm_dir)
            .with_context(|| format!("Failed to create SHM dir: {}", horus_shm_dir.display()))?;

        let path = horus_shm_dir.join(format!("horus_{}", name));

        let (file, is_owner) = if path.exists() {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .open(&path)
                .with_context(|| format!("Failed to open SHM: {}", path.display()))?;
            (file, false)
        } else {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(&path)
                .with_context(|| format!("Failed to create SHM: {}", path.display()))?;
            file.set_len(size as u64)?;
            (file, true)
        };

        // Acquire shared flock — see Linux impl for rationale.
        // SAFETY: file.as_raw_fd() is a valid open fd; LOCK_SH is a valid flock op.
        let flock_ret = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
        if flock_ret != 0 {
            anyhow::bail!(
                "Failed to acquire shared lock on SHM file '{}': {}",
                path.display(),
                std::io::Error::last_os_error()
            );
        }

        // SAFETY: file is valid with sufficient size
        let mut mmap = unsafe {
            MmapOptions::new()
                .len(size)
                .map_mut(&file)
                .with_context(|| format!("mmap failed for SHM: {}", path.display()))?
        };
        if is_owner {
            mmap.fill(0);
        }

        // As in `open_existing`. For the creator the `fill(0)` above has
        // already wired every page, so this restates the invariant cheaply.
        // SAFETY: `mmap` owns a live mapping of `size` bytes and outlives the
        // call; `make_resident` neither reads nor writes the region.
        unsafe { super::make_resident(mmap.as_ptr(), size) };

        Ok(Self {
            mmap,
            retired: Vec::new(),
            size,
            path,
            _file: file,
            owner: is_owner,
        })
    }

    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }

    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.mmap[..self.size]
    }

    #[inline]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        &mut self.mmap[..self.size]
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.size
    }

    #[inline]
    pub fn is_owner(&self) -> bool {
        self.owner
    }

    #[inline]
    pub fn backing_path(&self) -> &std::path::Path {
        &self.path
    }

    /// Grow the region to `new_size` bytes without synchronization.
    ///
    /// # Safety
    ///
    /// `&mut self` already excludes concurrent access to this struct's fields.
    /// What the caller still owes is about the *mapped bytes*: the grow publishes
    /// a mapping at a new address, so any pointer previously handed out by
    /// `as_ptr()` now addresses the retained older mapping. That is safe to read
    /// — the replaced mapping is kept alive and is a coherent view of the same
    /// file — but it describes the PRE-GROW geometry, so a caller must re-derive
    /// cached offsets before using the new slot layout.
    ///
    /// The previous contract here ("no other thread is concurrently reading ...
    /// guaranteed by the single-thread ownership contract and the migration
    /// lock") was false: `Topic` shares one region across clones on different
    /// threads and held no such lock. It was a live use-after-free.
    pub unsafe fn grow_unchecked(&mut self, new_size: usize) -> Result<()> {
        anyhow::ensure!(
            self.retired.len() < super::MAX_RETIRED_MAPPINGS,
            "refusing to grow: {} mappings already retained (cap {}); \
             see MAX_RETIRED_MAPPINGS",
            self.retired.len(),
            super::MAX_RETIRED_MAPPINGS
        );

        use memmap2::MmapOptions;

        anyhow::ensure!(
            new_size > self.size,
            "grow_unchecked: new_size ({}) must be > current size ({})",
            new_size,
            self.size
        );

        // Extend the backing file
        self._file.set_len(new_size as u64).with_context(|| {
            format!(
                "Failed to grow SHM file to {} bytes: {}",
                new_size,
                self.path.display()
            )
        })?;

        // Create a new mmap with the larger size
        let new_mmap = MmapOptions::new()
            .len(new_size)
            .map_mut(&self._file)
            .with_context(|| {
                format!(
                    "Failed to remap SHM at new size {}: {}",
                    new_size,
                    self.path.display()
                )
            })?;

        // Retain, do not drop: see the `retired` field.
        let old_mmap = std::mem::replace(&mut self.mmap, new_mmap);
        self.retired.push(old_mmap);
        self.size = new_size;

        // The remap is a brand-new mapping with empty page tables, so without
        // this every page of the grown region would fault again on first touch
        // — and a grow happens *because* a large message is arriving, i.e. at
        // the worst possible moment. Re-establish residency before anyone
        // publishes into it.
        // SAFETY: the new mapping is live and `new_size` bytes long; the call
        // neither reads nor writes it.
        unsafe { super::make_resident(self.mmap.as_ptr(), new_size) };

        Ok(())
    }
}

impl Drop for ShmRegion {
    fn drop(&mut self) {
        // Remove the backing file only if nobody else still has it open.
        //
        // "Owner" records only who won the race to create the file, which says
        // nothing about who is still using it. Removing on that basis alone
        // broke the ordinary publisher restart: the creator exited, took the
        // file with it, every subscriber stayed mapped to an orphaned inode that
        // nobody would write to again, and the next publisher created a fresh
        // file — two groups on different memory, no error on either side. This
        // is the same non-blocking LOCK_EX last-one-out test the Linux backend
        // uses; the LOCK_SH taken in `new()`/`open_existing()` is what it reads.
        //
        // Deliberately not gated on `self.owner`: whoever turns out to be last
        // does the cleanup, or a region whose creator left first would survive
        // until the filesystem is cleaned by hand.
        //
        // SAFETY: `_file` is open for the whole lifetime of `self`, so the fd is
        // valid here; LOCK_EX | LOCK_NB is a valid flock operation.
        let sole_holder =
            unsafe { libc::flock(self._file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) == 0 };
        if !sole_holder {
            // Another holder still has it mapped — leave the region alone.
            return;
        }

        if self.path.exists() {
            let _ = std::fs::remove_file(&self.path);
        }
        // The exclusive lock is released with the fd when `_file` drops.
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}
