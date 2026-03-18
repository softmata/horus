// Linux shared memory: /dev/shm tmpfs + mmap + flock
//
// Uses file-based mmap on /dev/shm (tmpfs) which is RAM-backed with no disk I/O.
// flock(LOCK_SH) is held for the lifetime of the region for stale detection.

use anyhow::{Context, Result};
use memmap2::{MmapMut, MmapOptions};
use std::fs::{File, OpenOptions};
use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt};
use std::os::unix::io::AsRawFd;
use std::path::PathBuf;

/// Cross-platform shared memory region for high-performance IPC.
///
/// Linux backend: tmpfs-backed files in `/dev/shm` (RAM).
#[derive(Debug)]
pub struct ShmRegion {
    mmap: MmapMut,
    _file: File,
    path: PathBuf,
    topic_name: String,
    size: usize,
    owner: bool,
}

impl ShmRegion {
    /// Create or open a shared memory region.
    pub fn new(name: &str, size: usize) -> Result<Self> {
        anyhow::ensure!(size > 0, "SHM region size must be > 0");
        anyhow::ensure!(!name.is_empty(), "SHM region name must not be empty");
        let horus_shm_dir = super::shm_topics_dir();
        // Mode 0o700: only the owner can list or access the SHM directory.
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(&horus_shm_dir)
            .with_context(|| {
                format!(
                    "Failed to create SHM directory: {}",
                    horus_shm_dir.display()
                )
            })?;

        let path = horus_shm_dir.join(format!("horus_{}", name));

        // Create parent directory if the name contains "/" (legacy topic names)
        if let Some(parent) = path.parent() {
            std::fs::DirBuilder::new()
                .recursive(true)
                .mode(0o700)
                .create(parent)
                .with_context(|| format!("Failed to create parent dir: {}", parent.display()))?;
        }

        // Atomically try to be the creator: create_new maps to O_CREAT|O_EXCL,
        // guaranteeing exactly one winner even when many threads race simultaneously.
        // Mode 0o600: owner read/write only.
        let (file, is_owner) = match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(&path)
        {
            Ok(file) => {
                if let Err(e) = file.set_len(size as u64) {
                    // Clean up the zero-length file on ENOSPC or other set_len failure
                    drop(file);
                    let _ = std::fs::remove_file(&path);
                    return Err(e.into());
                }
                (file, true)
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                let file = OpenOptions::new()
                    .read(true)
                    .write(true)
                    .open(&path)
                    .with_context(|| format!("Failed to open existing SHM: {}", path.display()))?;
                let metadata = file.metadata()?;
                if metadata.len() < size as u64 {
                    file.set_len(size as u64)?;
                }
                (file, false)
            }
            Err(e) => return Err(e.into()),
        };

        // Acquire a shared (read) flock on the backing file. Every process
        // that has this SHM region open holds LOCK_SH. When a process exits —
        // even via SIGKILL — the kernel closes the fd and releases the lock.
        // SAFETY: file.as_raw_fd() is a valid open fd; LOCK_SH is a valid flock op.
        let flock_ret = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
        if flock_ret != 0 {
            anyhow::bail!(
                "Failed to acquire shared lock on SHM file '{}': {}",
                path.display(),
                std::io::Error::last_os_error()
            );
        }

        // SAFETY: file is a valid open file with sufficient size; len(size) matches the file size
        let mut mmap = unsafe {
            MmapOptions::new()
                .len(size)
                .map_mut(&file)
                .with_context(|| format!("mmap failed for SHM: {}", path.display()))?
        };

        if is_owner {
            mmap.fill(0);
            // Write topic metadata file for discovery consistency across platforms
            let _ = super::write_topic_meta(name, size);
        }

        Ok(Self {
            mmap,
            size,
            path,
            topic_name: name.to_string(),
            _file: file,
            owner: is_owner,
        })
    }

    /// Raw pointer to the mapped memory.
    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }

    /// View the mapped memory as a byte slice.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.mmap[..self.size]
    }

    /// View the mapped memory as a mutable byte slice.
    #[inline]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        &mut self.mmap[..self.size]
    }

    /// Size of the mapped region in bytes.
    #[inline]
    pub fn len(&self) -> usize {
        self.size
    }

    /// Whether this handle is the original creator (responsible for cleanup on drop).
    #[inline]
    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// Path to the backing file (Linux-specific).
    #[inline]
    pub fn backing_path(&self) -> &std::path::Path {
        &self.path
    }
}

impl Drop for ShmRegion {
    fn drop(&mut self) {
        // flock(LOCK_SH) is automatically released when _file is dropped (fd closed).
        // We only need to unlink the file if we're the owner.
        if self.owner {
            super::remove_topic_meta(&self.topic_name);
            if self.path.exists() {
                let _ = std::fs::remove_file(&self.path);
            }
        }
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state;
// concurrent access is managed by atomic operations at the topic/transport layer
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

#[cfg(test)]
mod tests {
    use super::*;

    fn unique_name(prefix: &str) -> String {
        format!(
            "{}_{}_{}",
            prefix,
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        )
    }

    #[test]
    fn shm_create_and_basic_rw() {
        let name = unique_name("sys_basic");
        let size = 4096;
        let region = ShmRegion::new(&name, size).expect("Failed to create SHM region");
        assert!(region.is_owner());

        let ptr = region.as_ptr() as *mut u8;
        // SAFETY: ptr is a valid mmap'd region of `size` bytes; writes are within bounds
        unsafe {
            for i in 0..size {
                *ptr.add(i) = (i % 256) as u8;
            }
        }

        let rptr = region.as_ptr();
        for i in 0..size {
            // SAFETY: rptr is a valid mmap'd region of `size` bytes; i < size
            let val = unsafe { *rptr.add(i) };
            assert_eq!(val, (i % 256) as u8, "Mismatch at byte {}", i);
        }
    }

    #[test]
    fn shm_zero_initialized() {
        let name = unique_name("sys_zeroed");
        let region = ShmRegion::new(&name, 4096).expect("Failed to create SHM");
        for (i, &byte) in region.as_slice().iter().enumerate() {
            assert_eq!(byte, 0, "Byte {} not zeroed", i);
        }
    }

    #[test]
    fn shm_as_slice_rw() {
        let name = unique_name("sys_slice");
        let size = 256;
        let mut region = ShmRegion::new(&name, size).expect("create");
        assert_eq!(region.len(), size);

        let slice = region.as_slice_mut();
        for (i, byte) in slice.iter_mut().enumerate() {
            *byte = (i % 256) as u8;
        }

        let slice = region.as_slice();
        for (i, &byte) in slice.iter().enumerate() {
            assert_eq!(byte, (i % 256) as u8);
        }
    }

    #[test]
    fn shm_concurrent_creation_race() {
        use std::sync::{Arc, Barrier};

        let name = unique_name("sys_concurrent");
        let size = 4096;
        let n_threads = 4;

        let barrier = Arc::new(Barrier::new(n_threads));

        let results: Vec<_> = (0..n_threads)
            .map(|_| {
                let b = barrier.clone();
                let n = name.clone();
                std::thread::spawn(move || {
                    b.wait();
                    ShmRegion::new(&n, size)
                })
            })
            .collect();

        let outcomes: Vec<_> = results
            .into_iter()
            .map(|h| h.join().expect("thread panicked"))
            .collect();

        let (successes, failures): (Vec<_>, Vec<_>) = outcomes.iter().partition(|r| r.is_ok());
        assert_eq!(failures.len(), 0, "All threads must succeed");

        let owner_count = successes
            .iter()
            .filter(|r| r.as_ref().unwrap().is_owner())
            .count();
        assert_eq!(owner_count, 1, "Exactly 1 owner");
    }

    #[test]
    fn shm_drop_owner_removes_file() {
        let name = unique_name("sys_drop");
        let path;
        {
            let region = ShmRegion::new(&name, 4096).expect("create");
            path = region.backing_path().to_path_buf();
            assert!(path.exists());
        }
        assert!(!path.exists(), "Owner drop should remove file");
    }

    #[test]
    fn shm_non_owner_drop_does_not_remove_file() {
        let name = unique_name("sys_nonowner_drop");
        let region1 = ShmRegion::new(&name, 4096).expect("create owner");
        let path = region1.backing_path().to_path_buf();

        {
            let region2 = ShmRegion::new(&name, 4096).expect("open non-owner");
            assert!(!region2.is_owner());
        }

        assert!(
            path.exists(),
            "File should still exist after non-owner drop"
        );
        assert!(region1.is_owner());
    }

    #[test]
    fn shm_same_name_gets_same_region() {
        let name = unique_name("sys_reopen");
        let size = 4096;

        let region1 = ShmRegion::new(&name, size).expect("first create");
        // SAFETY: ptr is valid, writing first byte
        unsafe {
            let ptr = region1.as_ptr() as *mut u8;
            *ptr = 0xAB;
        }

        let region2 = ShmRegion::new(&name, size).expect("second open");
        assert!(!region2.is_owner());
        // SAFETY: ptr is valid, reading first byte
        unsafe {
            assert_eq!(*region2.as_ptr(), 0xAB, "Should see data from first");
        }
    }

    #[test]
    fn shm_flock_alive_while_region_exists() {
        let name = unique_name("sys_flock_alive");
        let region = ShmRegion::new(&name, 4096).expect("create");
        assert!(
            !crate::shm::is_shm_file_stale(region.backing_path()),
            "ShmRegion should hold flock → not stale"
        );
    }

    #[test]
    fn shm_flock_forget_keeps_lock() {
        let name = unique_name("sys_flock_forget");
        let region = ShmRegion::new(&name, 4096).expect("create");
        let path = region.backing_path().to_path_buf();

        std::mem::forget(region);

        assert!(path.exists());
        assert!(
            !crate::shm::is_shm_file_stale(&path),
            "Forgotten region still holds fd → flock → not stale"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn shm_file_permissions_owner_only() {
        use std::os::unix::fs::PermissionsExt;

        let name = unique_name("sys_perms");
        let region = ShmRegion::new(&name, 4096).expect("create");
        let metadata = std::fs::metadata(region.backing_path()).expect("metadata");
        let mode = metadata.permissions().mode() & 0o777;
        assert_eq!(mode, 0o600, "SHM file should be 0o600, got {:#o}", mode);
    }
}
