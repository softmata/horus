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
    _file: File,
    path: PathBuf,
    size: usize,
    owner: bool,
}

impl ShmRegion {
    /// Create or open a shared memory region.
    pub fn new(name: &str, size: usize) -> Result<Self> {
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

        Ok(Self {
            mmap,
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
}

impl Drop for ShmRegion {
    fn drop(&mut self) {
        if self.owner && self.path.exists() {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}
