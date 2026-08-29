// macOS shared memory: POSIX shm_open() + mmap (Mach shared memory, RAM-backed)

use anyhow::Result;
use std::os::unix::io::AsRawFd;
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

/// Take a shared lock that marks this process as a live holder of `name`.
///
/// This is the macOS answer to the Linux backend's last-one-out test (see
/// `shm/linux.rs`'s `Drop`): unlinking on creator-ownership alone orphans every
/// subscriber that outlives its publisher, and the restarted publisher then
/// creates a second, unrelated section with nothing on either side detecting it.
/// Darwin's `flock()` rejects a POSIX shm descriptor, so the shm fd cannot carry
/// the lock — the topic's `.meta` sidecar can, and every holder opens it.
///
/// Best-effort: the sidecar is only metadata, so a failure here degrades to the
/// old creator-only rule rather than failing the region outright.
fn acquire_holder_lock(name: &str, create: bool) -> Option<std::fs::File> {
    let path = super::shm_topics_dir().join(format!("{}.meta", super::sanitize_namespace(name)));
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(create)
        .create(create)
        .open(&path)
        .ok()?;
    // SAFETY: file.as_raw_fd() is a valid open fd; LOCK_SH is a valid flock op.
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) } != 0 {
        return None;
    }
    Some(file)
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
    /// Shared flock held for the region's lifetime so `Drop` can tell whether
    /// anyone else is still mapped. Kept alive until after that check.
    lock_file: Option<std::fs::File>,
}

impl ShmRegion {
    /// Open an existing region without creating it.
    pub fn open_existing(name: &str, minimum_size: usize) -> Result<Self> {
        anyhow::ensure!(minimum_size > 0, "SHM region size must be > 0");
        super::validate_region_name(name)?;
        use std::ffi::CString;
        let mut hash = 0xcbf29ce484222325u64;
        for byte in super::shm_namespace()
            .bytes()
            .chain([b'/'])
            .chain(name.bytes())
        {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x100000001b3);
        }
        let shm_name = format!("/horus_{hash:016x}");
        let c_name = CString::new(shm_name.clone())?;
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };
        if fd < 0 {
            anyhow::bail!(
                "shm '{}' does not exist: {}",
                shm_name,
                std::io::Error::last_os_error()
            );
        }
        let size = fstat_size(fd);
        if size < minimum_size as libc::off_t {
            unsafe { libc::close(fd) };
            anyhow::bail!("existing SHM region is too small");
        }
        // PROT_READ|PROT_WRITE, not PROT_READ. `ShmRegion` exposes one safe API
        // across four backends and `as_slice_mut()` promises a writable slice;
        // mapping read-only here made that slice a reference whose permissions
        // do not match the memory, so code that works on Linux took SIGBUS on
        // macOS at the first store. The fd above is already O_RDWR, so the
        // writable mapping is permitted.
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size as usize,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            unsafe { libc::close(fd) };
            anyhow::bail!("shm mmap failed: {}", std::io::Error::last_os_error());
        }
        // Pay the page faults at attach instead of inside the first receive
        // loop; see `horus_sys::shm::make_resident` for the policy and its
        // opt-outs.
        // SAFETY: `ptr` is a live mapping of `size` bytes that outlives the
        // call; `make_resident` neither reads nor writes the region.
        unsafe { super::make_resident(ptr as *const u8, size as usize) };

        // Join the holder set so the creator cannot unlink this region out from
        // under us. Attaching must never create the sidecar.
        let lock_file = acquire_holder_lock(name, false);
        Ok(Self {
            ptr: ptr as *mut u8,
            fd,
            shm_name,
            topic_name: name.to_string(),
            size: size as usize,
            owner: false,
            lock_file,
        })
    }

    /// Create or open a shared memory region using shm_open (RAM-backed).
    pub fn new(name: &str, size: usize) -> Result<Self> {
        anyhow::ensure!(size > 0, "SHM region size must be > 0");
        // `shm_open` takes a flat name, so traversal is not a filesystem concern
        // here — but the same names must be rejected on every platform so a
        // topic that is refused on Linux is not silently accepted on macOS.
        super::validate_region_name(name)?;
        use std::ffi::CString;

        // Darwin limits POSIX SHM names to PSHMNAMLEN (31 bytes). Namespaces
        // from CI plus descriptive topic names routinely exceed that limit, so
        // use a deterministic hash while metadata retains the original name.
        let mut hash = 0xcbf29ce484222325u64;
        for byte in super::shm_namespace()
            .bytes()
            .chain([b'/'])
            .chain(name.bytes())
        {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x100000001b3);
        }
        let shm_name = format!("/horus_{hash:016x}");
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

        // As in `open_existing`: wire the page tables now rather than one
        // minor fault at a time in the publish loop.
        // SAFETY: `ptr` is a live mapping of `size` bytes that outlives the
        // call; `make_resident` neither reads nor writes the region.
        unsafe { super::make_resident(ptr as *const u8, size) };

        // Every holder — creator or not — keeps a shared lock on the sidecar for
        // the lifetime of the region; that is what makes the last-one-out test
        // in `Drop` correct.
        let lock_file = acquire_holder_lock(name, is_owner);

        Ok(Self {
            ptr: ptr as *mut u8,
            fd,
            shm_name,
            topic_name: name.to_string(),
            size,
            owner: is_owner,
            lock_file,
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

    /// Grow the region to `new_size` bytes without synchronization.
    ///
    /// # Safety
    ///
    /// The caller must ensure no other thread is concurrently reading from or
    /// writing to this memory region via raw pointers derived from `as_ptr()`.
    pub unsafe fn grow_unchecked(&mut self, new_size: usize) -> Result<()> {
        anyhow::ensure!(
            new_size > self.size,
            "grow_unchecked: new_size ({}) must be > current size ({})",
            new_size,
            self.size
        );

        // Extend the backing POSIX shared memory object
        let ret = libc::ftruncate(self.fd, new_size as libc::off_t);
        anyhow::ensure!(
            ret == 0,
            "ftruncate failed: {}",
            std::io::Error::last_os_error()
        );

        // Map the larger region FIRST; the old mapping stays valid until it
        // succeeds. Tearing the old one down first meant the `ensure!` below
        // returned with `self.ptr`/`self.size` still describing address space
        // that had just been unmapped, so `as_ptr()`/`as_slice()`/
        // `as_slice_mut()` handed out dangling pointers and `Drop` munmapped the
        // range a second time. Two MAP_SHARED views of the same shm fd may
        // coexist, so nothing requires the munmap to come first. This matches
        // the Linux and Windows backends, which already map before releasing.
        let new_ptr = libc::mmap(
            std::ptr::null_mut(),
            new_size,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            self.fd,
            0,
        );
        anyhow::ensure!(
            new_ptr != libc::MAP_FAILED,
            "mmap after grow failed: {}",
            std::io::Error::last_os_error()
        );

        // Only now release the old mapping.
        libc::munmap(self.ptr as *mut libc::c_void, self.size);

        self.ptr = new_ptr as *mut u8;
        self.size = new_size;

        // The remap is a brand-new mapping with empty page tables, so without
        // this every page of the grown region would fault again on first touch
        // — and a grow happens *because* a large message is arriving, i.e. at
        // the worst possible moment. Re-establish residency before anyone
        // publishes into it.
        // SAFETY: the new mapping is live and `new_size` bytes long; the call
        // neither reads nor writes it.
        super::make_resident(self.ptr as *const u8, new_size);

        Ok(())
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

        // Unlink only if nobody else still has this region open.
        //
        // "Owner" records only who won the race to create the section, which
        // says nothing about who is still using it. Unlinking on that basis
        // alone broke the ordinary publisher restart: the creator exited, took
        // the name with it, every subscriber stayed mapped to a section nobody
        // would write to again, and the restarted publisher's
        // `shm_open(O_CREAT|O_EXCL)` succeeded against the freed name and
        // created a *second* one. Neither side saw an error. The Linux backend
        // fixed this with a non-blocking LOCK_EX last-one-out test; this is the
        // same test, taken on the `.meta` sidecar because Darwin's `flock()`
        // rejects a POSIX shm descriptor.
        //
        // Deliberately not gated on `self.owner`: whoever turns out to be last
        // does the cleanup, or a region whose creator left first would survive
        // until reboot.
        let sole_holder = match &self.lock_file {
            // SAFETY: the File is open for the whole lifetime of `self`, so the
            // fd is valid here; LOCK_EX | LOCK_NB is a valid flock operation.
            Some(f) => unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) == 0 },
            // No sidecar lock available: fall back to the old creator rule
            // rather than leaking the region forever.
            None => self.owner,
        };
        if !sole_holder {
            return;
        }

        super::remove_topic_meta(&self.topic_name);
        if let Ok(c_name) = std::ffi::CString::new(self.shm_name.clone()) {
            // SAFETY: c_name is a valid null-terminated CString
            unsafe { libc::shm_unlink(c_name.as_ptr()) };
        }
        // The exclusive lock is released with the fd when `lock_file` drops.
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}
