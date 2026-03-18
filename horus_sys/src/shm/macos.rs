// macOS shared memory: POSIX shm_open() + mmap (Mach shared memory, RAM-backed)

use anyhow::Result;
use std::time::Duration;

/// Poll the size of an open SHM fd via fstat.
fn fstat_size(fd: i32) -> libc::off_t {
    // SAFETY: zeroed libc::stat is a valid all-zero value for the POD struct.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    // SAFETY: fd is a valid open file descriptor owned by the caller.
    if unsafe { libc::fstat(fd, &mut st) } == 0 {
        st.st_size
    } else {
        -1
    }
}

/// Wait for a non-owner SHM fd to reach the expected size.
///
/// There is a partial-initialization window between the owner calling
/// `shm_open(O_CREAT)` and the subsequent `ftruncate()`. A non-owner
/// that opens the object in this window sees `st_size == 0` and must
/// not call `mmap()` yet.
///
/// Retries with exponential backoff (1 ms base, up to 10 attempts).
fn wait_for_shm_init(fd: i32, expected_size: usize) -> std::result::Result<(), ()> {
    const SHM_INIT_MAX_RETRIES: u32 = 10;

    for retry in 0..SHM_INIT_MAX_RETRIES {
        if fstat_size(fd) >= expected_size as libc::off_t {
            return Ok(());
        }
        let delay_ms = 1u64 << retry.min(6);
        std::thread::sleep(Duration::from_millis(delay_ms));
    }

    Err(())
}

/// Cross-platform shared memory region for high-performance IPC.
///
/// macOS backend: POSIX `shm_open()` (Mach shared memory, RAM-backed).
#[derive(Debug)]
pub struct ShmRegion {
    ptr: *mut u8,
    fd: i32,
    shm_name: String,
    topic_name: String,
    size: usize,
    owner: bool,
}

impl ShmRegion {
    /// Create or open a shared memory region using shm_open (RAM-backed).
    pub fn new(name: &str, size: usize) -> Result<Self> {
        anyhow::ensure!(size > 0, "SHM region size must be > 0");
        anyhow::ensure!(!name.is_empty(), "SHM region name must not be empty");
        use std::ffi::CString;

        let shm_name = format!("/horus_{}_{}", super::shm_namespace(), name);
        let c_name = CString::new(shm_name.clone())
            .map_err(|e| anyhow::anyhow!("Invalid shm name '{}': {}", shm_name, e))?;

        // Try to open existing first
        // SAFETY: c_name is a valid null-terminated CString; flags are valid POSIX constants
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };

        let (fd, is_owner) = if fd >= 0 {
            if wait_for_shm_init(fd, size).is_ok() {
                (fd, false)
            } else {
                // Zombie: creator died between shm_open(O_CREAT) and ftruncate.
                // SAFETY: fd is a valid open file descriptor
                unsafe { libc::close(fd) };
                // SAFETY: c_name is a valid null-terminated CString
                unsafe { libc::shm_unlink(c_name.as_ptr()) };

                // SAFETY: O_CREAT|O_RDWR|O_EXCL are valid POSIX flags
                let new_fd = unsafe {
                    libc::shm_open(
                        c_name.as_ptr(),
                        libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                        0o600,
                    )
                };
                if new_fd >= 0 {
                    // SAFETY: new_fd is a valid open file descriptor
                    if unsafe { libc::ftruncate(new_fd, size as libc::off_t) } != 0 {
                        unsafe { libc::close(new_fd) };
                        unsafe { libc::shm_unlink(c_name.as_ptr()) };
                        anyhow::bail!(
                            "Failed to set shm '{}' to {} bytes after zombie reclaim: {}",
                            shm_name,
                            size,
                            std::io::Error::last_os_error()
                        );
                    }
                    (new_fd, true)
                } else {
                    // Lost re-create race — open their copy
                    // SAFETY: c_name is a valid null-terminated CString
                    let retry_fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };
                    if retry_fd < 0 {
                        anyhow::bail!(
                            "Failed to open/create shm '{}' after zombie cleanup: {}",
                            shm_name,
                            std::io::Error::last_os_error()
                        );
                    }
                    if wait_for_shm_init(retry_fd, size).is_err() {
                        unsafe { libc::close(retry_fd) };
                        anyhow::bail!("shm '{}' not fully initialized after all retries", shm_name);
                    }
                    (retry_fd, false)
                }
            }
        } else {
            // Create new
            // SAFETY: c_name is a valid null-terminated CString
            let fd = unsafe {
                libc::shm_open(
                    c_name.as_ptr(),
                    libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                    0o600,
                )
            };
            if fd < 0 {
                // Race: someone else created it — open their copy
                let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };
                if fd < 0 {
                    anyhow::bail!(
                        "Failed to open/create shm '{}': {}",
                        shm_name,
                        std::io::Error::last_os_error()
                    );
                }
                if wait_for_shm_init(fd, size).is_err() {
                    unsafe { libc::close(fd) };
                    anyhow::bail!("shm '{}' not fully initialized after all retries", shm_name);
                }
                (fd, false)
            } else {
                // SAFETY: fd is a valid open file descriptor
                if unsafe { libc::ftruncate(fd, size as libc::off_t) } != 0 {
                    unsafe { libc::close(fd) };
                    unsafe { libc::shm_unlink(c_name.as_ptr()) };
                    anyhow::bail!(
                        "Failed to set shm '{}' size to {} bytes: {}",
                        shm_name,
                        size,
                        std::io::Error::last_os_error()
                    );
                }
                (fd, true)
            }
        };

        // Memory map the shared memory
        // SAFETY: fd is valid, size > 0, flags are valid POSIX mmap constants
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };

        if ptr == libc::MAP_FAILED {
            unsafe { libc::close(fd) };
            if is_owner {
                unsafe { libc::shm_unlink(c_name.as_ptr()) };
            }
            anyhow::bail!("shm mmap failed: {}", std::io::Error::last_os_error());
        }

        if is_owner {
            // SAFETY: ptr is valid from mmap, size matches the mapped region
            unsafe {
                std::ptr::write_bytes(ptr as *mut u8, 0, size);
            }
            // Write topic metadata file so discovery can find this SHM region
            let _ = super::write_topic_meta(name, size);
        }

        Ok(Self {
            ptr: ptr as *mut u8,
            fd,
            shm_name,
            topic_name: name.to_string(),
            size,
            owner: is_owner,
        })
    }

    /// Raw pointer to the mapped memory.
    #[inline]
    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }

    /// View the mapped memory as a byte slice.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        // SAFETY: ptr is valid from mmap, size bytes are mapped
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }

    /// View the mapped memory as a mutable byte slice.
    #[inline]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        // SAFETY: ptr is valid from mmap, size bytes are mapped, &mut self ensures exclusive
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.size) }
    }

    /// Size of the mapped region in bytes.
    #[inline]
    pub fn len(&self) -> usize {
        self.size
    }

    /// Whether this handle is the original creator.
    #[inline]
    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// No filesystem backing path on macOS (uses POSIX shm_open).
    #[inline]
    pub fn backing_path(&self) -> Option<&std::path::Path> {
        None
    }
}

impl Drop for ShmRegion {
    fn drop(&mut self) {
        // SAFETY: self.ptr is a valid mmap'd pointer, self.size matches,
        // self.fd is a valid open file descriptor
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.size);
            libc::close(self.fd);
        }

        if self.owner {
            super::remove_topic_meta(&self.topic_name);
            if let Ok(c_name) = std::ffi::CString::new(self.shm_name.clone()) {
                // SAFETY: c_name is a valid null-terminated CString
                unsafe { libc::shm_unlink(c_name.as_ptr()) };
            }
        }
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}
