// HORUS Shared Memory Region - Cross-platform optimized shared memory
//
// Each platform uses its optimal shared memory mechanism:
// - Linux: /dev/shm files (tmpfs - RAM-backed, already optimal)
// - macOS: shm_open() + mmap (POSIX shared memory, RAM-backed via Mach)
// - Windows: CreateFileMappingW with INVALID_HANDLE_VALUE (pagefile-backed, optimized for IPC)

#[cfg(not(target_os = "linux"))]
use crate::error::HorusError;
use crate::error::HorusResult;
use std::path::PathBuf;

#[cfg(target_os = "linux")]
use crate::memory::platform::shm_topics_dir;
#[cfg(target_os = "linux")]
use memmap2::{MmapMut, MmapOptions};
#[cfg(target_os = "linux")]
use std::fs::{File, OpenOptions};
#[cfg(target_os = "linux")]
use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt};

/// Cross-platform shared memory region for high-performance IPC
///
/// Uses the optimal shared memory mechanism for each platform:
/// - Linux: tmpfs-backed files in /dev/shm (RAM)
/// - macOS: POSIX shm_open() (Mach shared memory, RAM)
/// - Windows: CreateFileMapping with page file backing (optimized IPC)
#[derive(Debug)]
pub struct ShmRegion {
    #[cfg(target_os = "linux")]
    mmap: MmapMut,
    #[cfg(target_os = "linux")]
    _file: File,
    #[cfg(target_os = "linux")]
    path: PathBuf,

    #[cfg(target_os = "macos")]
    ptr: *mut u8,
    #[cfg(target_os = "macos")]
    fd: i32,
    #[cfg(target_os = "macos")]
    shm_name: String,

    #[cfg(target_os = "windows")]
    ptr: *mut u8,
    #[cfg(target_os = "windows")]
    handle: isize, // HANDLE

    #[allow(dead_code)] // used on macOS/Windows for Drop; stored on Linux for future use
    size: usize,
    owner: bool,
}

// ============================================================================
// Linux Implementation - File-based mmap on /dev/shm (tmpfs)
// Already optimal - tmpfs is RAM-backed with no disk I/O
// ============================================================================

#[cfg(target_os = "linux")]
impl ShmRegion {
    /// Create or open a shared memory region
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        // Use flat namespace - all topics in same directory (ROS-like simplicity)
        let horus_shm_dir = shm_topics_dir();
        // Mode 0o700: only the owner can list or access the SHM directory.
        // Other local users must not be able to enumerate running nodes/topics.
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(&horus_shm_dir)?;

        // Topic names use dot notation (e.g., "motors.cmd_vel") - no conversion needed
        // Names can also contain "/" for namespacing (e.g., "links/sensor_test")
        let path = horus_shm_dir.join(format!("horus_{}", name));

        // Create parent directory if the name contains "/" (e.g., "links/sensor_test")
        if let Some(parent) = path.parent() {
            std::fs::DirBuilder::new()
                .recursive(true)
                .mode(0o700)
                .create(parent)?;
        }

        // Atomically try to be the creator: `create_new` maps to O_CREAT|O_EXCL,
        // guaranteeing exactly one winner even when many threads race simultaneously.
        // The previous path.exists() + create(true) pattern had a TOCTOU window
        // where all racing threads could see the file absent and all become "owner".
        // Mode 0o600: owner read/write only — no world-readable SHM files.
        let (file, is_owner) = match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(&path)
        {
            Ok(file) => {
                // We created the file — set its size and become the owner.
                file.set_len(size as u64)?;
                (file, true)
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                // Another thread/process created it first — open normally.
                let file = OpenOptions::new().read(true).write(true).open(&path)?;
                let metadata = file.metadata()?;
                if metadata.len() < size as u64 {
                    file.set_len(size as u64)?;
                }
                (file, false)
            }
            Err(e) => return Err(e.into()),
        };

        // SAFETY: file is a valid open file with sufficient size set above; len(size) matches the file size
        let mut mmap = unsafe { MmapOptions::new().len(size).map_mut(&file)? };

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

    pub fn as_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }
}

#[cfg(target_os = "linux")]
impl Drop for ShmRegion {
    fn drop(&mut self) {
        if self.owner && self.path.exists() {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

// ============================================================================
// macOS Implementation - POSIX shm_open() (Mach shared memory, RAM-backed)
// Much faster than /tmp file-based approach
// ============================================================================

/// Poll the size of an open SHM fd via `fstat`.  Returns the file size
/// in bytes, or -1 on fstat failure.
#[cfg(target_os = "macos")]
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
/// `shm_open(O_CREAT)` and the subsequent `ftruncate()`.  A non-owner
/// that opens the object in this window sees `st_size == 0` and must
/// not call `mmap()` yet — doing so either fails or maps nothing.
///
/// This function retries with exponential backoff (1 ms base, up to
/// `SHM_INIT_MAX_RETRIES` attempts) before giving up.
///
/// Returns `Ok(())` when the object has been fully sized, or `Err(())`
/// if the owner appears to have crashed before completing initialization.
#[cfg(target_os = "macos")]
fn wait_for_shm_init(fd: i32, expected_size: usize) -> Result<(), ()> {
    const SHM_INIT_MAX_RETRIES: u32 = 10;

    for retry in 0..SHM_INIT_MAX_RETRIES {
        if fstat_size(fd) >= expected_size as libc::off_t {
            return Ok(());
        }
        // Exponential backoff: 1, 2, 4, 8, 16, 32, 64 ms (capped).
        let delay_ms = 1u64 << retry.min(6);
        std::thread::sleep(std::time::Duration::from_millis(delay_ms));
    }

    Err(())
}

#[cfg(target_os = "macos")]
impl ShmRegion {
    /// Create or open a shared memory region using shm_open (RAM-backed)
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        use std::ffi::CString;

        // Include SHM namespace so multiple HORUS instances can coexist on the
        // same machine without shm_open name collisions.  The namespace is
        // always non-empty (auto-generated from PGID+UID when no env var is set).
        let shm_name = format!(
            "/horus_{}_{}",
            crate::memory::platform::shm_namespace(),
            name
        );
        let c_name = CString::new(shm_name.clone()).map_err(|e| {
            HorusError::Memory(
                format!(
                    "Invalid shm name '{}': topic names cannot contain null bytes: {}",
                    shm_name, e
                )
                .into(),
            )
        })?;

        // Try to open existing first
        // SAFETY: c_name is a valid null-terminated CString; flags are valid POSIX constants
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };

        let (fd, is_owner) = if fd >= 0 {
            // Opened existing — verify size before mmap.
            //
            // The SHM object may have just been created by another process
            // that hasn't called ftruncate() yet (size == 0). mmapping a
            // zero-size object either fails or maps nothing, causing a crash.
            // Retry with exponential backoff; if still wrong after all retries,
            // the creator crashed mid-setup — reclaim the zombie object.
            if wait_for_shm_init(fd, size).is_ok() {
                (fd, false)
            } else {
                // Zombie: creator died between shm_open(O_CREAT) and ftruncate.
                // Reclaim: close fd, unlink the object, then re-create.
                // SAFETY: fd is a valid open file descriptor
                unsafe { libc::close(fd) };
                // SAFETY: c_name is a valid null-terminated CString
                unsafe { libc::shm_unlink(c_name.as_ptr()) };

                // Now try to become the new owner.
                // SAFETY: O_CREAT|O_RDWR|O_EXCL are valid POSIX flags
                let new_fd = unsafe {
                    libc::shm_open(
                        c_name.as_ptr(),
                        libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                        0o600,
                    )
                };
                if new_fd >= 0 {
                    // Won the re-create race — set the size.
                    // SAFETY: new_fd is a valid open file descriptor
                    if unsafe { libc::ftruncate(new_fd, size as libc::off_t) } != 0 {
                        unsafe { libc::close(new_fd) };
                        unsafe { libc::shm_unlink(c_name.as_ptr()) };
                        return Err(HorusError::Memory(
                            format!(
                                "Failed to set shm '{}' to {} bytes after zombie reclaim: {}",
                                shm_name,
                                size,
                                std::io::Error::last_os_error()
                            )
                            .into(),
                        ));
                    }
                    (new_fd, true)
                } else {
                    // Lost the re-create race to yet another process — open their copy.
                    // SAFETY: c_name is a valid null-terminated CString
                    let retry_fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };
                    if retry_fd < 0 {
                        return Err(HorusError::Memory(
                            format!(
                                "Failed to open/create shm '{}' after zombie cleanup: {}",
                                shm_name,
                                std::io::Error::last_os_error()
                            )
                            .into(),
                        ));
                    }
                    // The new owner is responsible for sizing; wait for it.
                    if wait_for_shm_init(retry_fd, size).is_err() {
                        unsafe { libc::close(retry_fd) };
                        return Err(HorusError::Memory(
                            format!("shm '{}' not fully initialized after all retries", shm_name)
                                .into(),
                        ));
                    }
                    (retry_fd, false)
                }
            }
        } else {
            // Create new
            // SAFETY: c_name is a valid null-terminated CString; O_CREAT|O_RDWR|O_EXCL are valid POSIX flags
            let fd = unsafe {
                libc::shm_open(
                    c_name.as_ptr(),
                    libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                    0o600,
                )
            };
            if fd < 0 {
                // Race condition: someone else created it between our two shm_open calls.
                // Open their copy and wait for them to finish ftruncate.
                // SAFETY: c_name is still a valid null-terminated CString
                let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o600) };
                if fd < 0 {
                    return Err(HorusError::Memory(
                        format!(
                            "Failed to open/create shm '{}': {}",
                            shm_name,
                            std::io::Error::last_os_error()
                        )
                        .into(),
                    ));
                }
                // Wait for the winner to call ftruncate before we mmap.
                if wait_for_shm_init(fd, size).is_err() {
                    // SAFETY: fd is a valid open file descriptor
                    unsafe { libc::close(fd) };
                    return Err(HorusError::Memory(
                        format!(
                            "shm '{}' not fully initialized after all retries: \
                         winner process may have crashed mid-setup",
                            shm_name
                        )
                        .into(),
                    ));
                }
                (fd, false)
            } else {
                // Set size for new region
                // SAFETY: fd is a valid open file descriptor from shm_open above
                if unsafe { libc::ftruncate(fd, size as libc::off_t) } != 0 {
                    // SAFETY: fd is a valid open file descriptor
                    unsafe { libc::close(fd) };
                    // SAFETY: c_name is a valid null-terminated CString
                    unsafe { libc::shm_unlink(c_name.as_ptr()) };
                    return Err(HorusError::Memory(format!(
                        "Failed to set shm '{}' to {} bytes: {} (macOS: check available memory with 'vm_stat')",
                        shm_name, size, std::io::Error::last_os_error()
                    ).into()));
                }
                (fd, true)
            }
        };

        // Memory map the shared memory
        // SAFETY: fd is valid, size > 0, and flags are valid POSIX mmap constants
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
            // SAFETY: fd is a valid open file descriptor
            unsafe { libc::close(fd) };
            if is_owner {
                // SAFETY: c_name is a valid null-terminated CString
                unsafe { libc::shm_unlink(c_name.as_ptr()) };
            }
            return Err(HorusError::Memory(
                format!("Failed to mmap shm: {}", std::io::Error::last_os_error()).into(),
            ));
        }

        // Initialize to zero if owner
        if is_owner {
            // SAFETY: ptr is valid from mmap (MAP_FAILED checked above), size matches the mapped region
            unsafe {
                std::ptr::write_bytes(ptr as *mut u8, 0, size);
            }
        }

        Ok(Self {
            ptr: ptr as *mut u8,
            fd,
            shm_name,
            size,

            owner: is_owner,
        })
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }
}

#[cfg(target_os = "macos")]
impl Drop for ShmRegion {
    fn drop(&mut self) {
        // Unmap memory
        // SAFETY: self.ptr is a valid mmap'd pointer and self.size matches the mapped region;
        // self.fd is a valid open file descriptor
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.size);
            libc::close(self.fd);
        }

        // Unlink if owner
        if self.owner {
            if let Ok(c_name) = std::ffi::CString::new(self.shm_name.clone()) {
                // SAFETY: c_name is a valid null-terminated CString
                unsafe { libc::shm_unlink(c_name.as_ptr()) };
            }
        }
    }
}

// ============================================================================
// Windows Implementation - CreateFileMappingW with pagefile backing
// Uses INVALID_HANDLE_VALUE for pure shared memory (no temp files)
// ============================================================================

#[cfg(target_os = "windows")]
impl ShmRegion {
    /// Create or open a shared memory region using Windows API (pagefile-backed)
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        use windows_sys::Win32::Foundation::{
            CloseHandle, GetLastError, ERROR_ALREADY_EXISTS, INVALID_HANDLE_VALUE,
        };
        use windows_sys::Win32::System::Memory::{
            CreateFileMappingW, MapViewOfFile, FILE_MAP_ALL_ACCESS, PAGE_READWRITE,
        };

        // Use flat namespace - all topics share same prefix (ROS-like simplicity)
        let mapping_name = format!("Local\\horus_{}", name);

        // Convert to wide string
        let wide_name: Vec<u16> = mapping_name
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();

        // Create or open file mapping (INVALID_HANDLE_VALUE = pagefile-backed)
        // SAFETY: INVALID_HANDLE_VALUE creates pagefile-backed mapping; wide_name is a valid
        // null-terminated wide string; PAGE_READWRITE and size parameters are valid
        let handle = unsafe {
            CreateFileMappingW(
                INVALID_HANDLE_VALUE as isize,
                std::ptr::null(),
                PAGE_READWRITE,
                (size >> 32) as u32, // High DWORD
                size as u32,         // Low DWORD
                wide_name.as_ptr(),
            )
        };

        if handle == 0 {
            return Err(HorusError::Memory(
                format!(
                    "CreateFileMappingW failed: error {}",
                    // SAFETY: GetLastError is always safe to call after a Windows API failure
                    unsafe { GetLastError() }
                )
                .into(),
            ));
        }

        // SAFETY: GetLastError is always safe to call; checks if mapping already existed
        let is_owner = unsafe { GetLastError() } != ERROR_ALREADY_EXISTS;

        // Map view of file
        // SAFETY: handle is a valid file mapping from CreateFileMappingW (non-zero checked above)
        let ptr = unsafe { MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size) };

        if ptr.is_null() {
            // SAFETY: handle is a valid file mapping handle from CreateFileMappingW
            unsafe { CloseHandle(handle) };
            return Err(HorusError::Memory(
                format!(
                    "MapViewOfFile failed: error {}",
                    // SAFETY: GetLastError is always safe to call after a Windows API failure
                    unsafe { GetLastError() }
                )
                .into(),
            ));
        }

        // Initialize to zero if owner
        if is_owner {
            // SAFETY: ptr is valid from MapViewOfFile (null checked above), size matches the mapped region
            unsafe {
                std::ptr::write_bytes(ptr as *mut u8, 0, size);
            }
        }

        Ok(Self {
            ptr: ptr as *mut u8,
            handle,
            size,

            owner: is_owner,
        })
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }
}

#[cfg(target_os = "windows")]
impl Drop for ShmRegion {
    fn drop(&mut self) {
        use windows_sys::Win32::Foundation::CloseHandle;
        use windows_sys::Win32::System::Memory::UnmapViewOfFile;

        // SAFETY: self.ptr is a valid mapped view from MapViewOfFile;
        // self.handle is a valid file mapping handle from CreateFileMappingW/OpenFileMappingW
        unsafe {
            UnmapViewOfFile(self.ptr as *const std::ffi::c_void);
            CloseHandle(self.handle);
        }
        // Note: Windows automatically cleans up named file mappings when all handles are closed
    }
}

// Common accessors — `size` and `owner` fields exist on all platform variants.
impl ShmRegion {
    /// Whether this handle is the original creator (responsible for cleanup on drop).
    pub fn is_owner(&self) -> bool {
        self.owner
    }
}

// Thread safety - shared memory regions can be sent between threads
// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state;
// concurrent access is managed by atomic operations at the topic/transport layer
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

// ============================================================================
// Tests
// ============================================================================

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
        let name = unique_name("test_basic");
        let size = 4096;
        let region = ShmRegion::new(&name, size).expect("Failed to create SHM region");
        assert!(region.is_owner());

        // Write pattern via as_ptr (the mmap is PROT_READ|PROT_WRITE)
        let ptr = region.as_ptr() as *mut u8;
        unsafe {
            for i in 0..size {
                *ptr.add(i) = (i % 256) as u8;
            }
        }

        // Read back and verify
        let rptr = region.as_ptr();
        for i in 0..size {
            let val = unsafe { *rptr.add(i) };
            assert_eq!(val, (i % 256) as u8, "Mismatch at byte {}", i);
        }
    }

    #[test]
    fn shm_zero_initialized() {
        let name = unique_name("test_zeroed");
        let size = 4096;

        let region = ShmRegion::new(&name, size).expect("Failed to create SHM");
        let ptr = region.as_ptr();

        // Owner should zero-initialize
        for i in 0..size {
            let val = unsafe { *ptr.add(i) };
            assert_eq!(val, 0, "Byte {} not zeroed", i);
        }
    }

    /// Two threads race to create the same SHM region.  Both must end up
    /// with a valid, fully-sized mapping — no crash, no mmap of a zero-size object.
    #[test]
    fn shm_concurrent_creation_race() {
        use std::sync::{Arc, Barrier};

        let name = unique_name("test_concurrent_create");
        let size = 4096;
        let n_threads = 4;

        // Synchronize all threads to start simultaneously, maximizing the race window.
        let barrier = Arc::new(Barrier::new(n_threads));

        let results: Vec<_> = (0..n_threads)
            .map(|_| {
                let b = barrier.clone();
                let n = name.clone();
                std::thread::spawn(move || {
                    b.wait(); // all threads start at the same time
                    ShmRegion::new(&n, size)
                })
            })
            .collect();

        let outcomes: Vec<_> = results
            .into_iter()
            .map(|h| h.join().expect("thread panicked"))
            .collect();

        // Every thread must have succeeded.
        let (successes, failures): (Vec<_>, Vec<_>) = outcomes.iter().partition(|r| r.is_ok());
        assert_eq!(
            failures.len(),
            0,
            "{}/{} threads failed to create/open SHM; errors: {:?}",
            failures.len(),
            n_threads,
            failures
        );

        // Exactly one thread must be the owner.
        let owner_count = successes
            .iter()
            .filter(|r| r.as_ref().unwrap().is_owner())
            .count();
        assert_eq!(
            owner_count, 1,
            "expected exactly 1 owner thread, got {}",
            owner_count
        );

        // All regions must be the correct size and accessible.
        for region in successes.iter().map(|r| r.as_ref().unwrap()) {
            let ptr = region.as_ptr();
            // Write and read-back a single byte to confirm the mapping is live.
            unsafe {
                let p = ptr as *mut u8;
                *p = 0xAB;
                assert_eq!(*p, 0xAB, "mapped region is not readable/writable");
            }
        }
    }

    #[test]
    fn shm_concurrent_rw_from_threads() {
        use std::sync::atomic::{AtomicU64, Ordering};
        use std::sync::Arc;

        let name = unique_name("test_threaded");
        let size = 4096;

        let region = Arc::new(ShmRegion::new(&name, size).expect("Failed to create SHM"));

        // Use the first 8 bytes as an atomic counter
        let counter_ptr = region.as_ptr() as *const AtomicU64;

        let n_threads = 4;
        let n_increments = 1000;

        let handles: Vec<_> = (0..n_threads)
            .map(|_| {
                let r = region.clone();
                std::thread::spawn(move || {
                    let counter = unsafe { &*(r.as_ptr() as *const AtomicU64) };
                    for _ in 0..n_increments {
                        counter.fetch_add(1, Ordering::Relaxed);
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        let final_val = unsafe { (*counter_ptr).load(Ordering::Relaxed) };
        assert_eq!(
            final_val,
            (n_threads * n_increments) as u64,
            "Concurrent atomic increments should sum correctly"
        );
    }

    // ── Negative input tests ──────────────────────────────────────────────

    #[test]
    fn shm_small_size() {
        let name = unique_name("test_small");
        // Minimum viable size — 1 byte
        let region = ShmRegion::new(&name, 1).expect("1-byte SHM should succeed");
        assert!(region.is_owner());
        // Write and read 1 byte
        unsafe {
            let ptr = region.as_ptr() as *mut u8;
            *ptr = 0xFF;
            assert_eq!(*ptr, 0xFF);
        }
    }

    #[test]
    fn shm_page_aligned_size() {
        let name = unique_name("test_page");
        // Exactly one page (4096)
        let region = ShmRegion::new(&name, 4096).expect("page-size SHM should succeed");
        assert!(region.is_owner());
    }

    #[test]
    fn shm_non_page_aligned_size() {
        let name = unique_name("test_non_page");
        // Non-page-aligned: 5000 bytes
        let region = ShmRegion::new(&name, 5000).expect("non-page-aligned SHM should succeed");
        // Should be able to write to all 5000 bytes
        let ptr = region.as_ptr() as *mut u8;
        unsafe {
            *ptr.add(4999) = 0xAB;
            assert_eq!(*ptr.add(4999), 0xAB);
        }
    }

    #[test]
    fn shm_same_name_gets_same_region() {
        let name = unique_name("test_reopen");
        let size = 4096;

        let region1 = ShmRegion::new(&name, size).expect("first create");
        assert!(region1.is_owner());

        // Write a pattern via region1
        unsafe {
            let ptr = region1.as_ptr() as *mut u8;
            *ptr = 0xAB;
        }

        let region2 = ShmRegion::new(&name, size).expect("second open");
        assert!(!region2.is_owner(), "Second opener should not be owner");

        // Should see the same data
        unsafe {
            let ptr = region2.as_ptr();
            assert_eq!(*ptr, 0xAB, "Second region should see data from first");
        }
    }

    #[test]
    fn shm_drop_owner_removes_file() {
        let name = unique_name("test_drop");
        let size = 4096;

        let path;
        {
            let region = ShmRegion::new(&name, size).expect("create");
            path = region.path.clone();
            assert!(path.exists(), "SHM file should exist while region is alive");
        }
        // After drop, owner should clean up the file
        assert!(
            !path.exists(),
            "SHM file should be removed after owner drops"
        );
    }

    #[test]
    fn shm_non_owner_drop_does_not_remove_file() {
        let name = unique_name("test_non_owner_drop");
        let size = 4096;

        let region1 = ShmRegion::new(&name, size).expect("create owner");
        let path = region1.path.clone();

        {
            let region2 = ShmRegion::new(&name, size).expect("open non-owner");
            assert!(!region2.is_owner());
            // region2 drops here — should NOT remove the file
        }

        assert!(
            path.exists(),
            "SHM file should still exist after non-owner drops"
        );

        // Owner's data should still be intact
        assert!(region1.is_owner());
        // region1 drops here and cleans up
    }

    #[test]
    fn shm_file_permissions_owner_only() {
        use std::os::unix::fs::PermissionsExt;

        let name = unique_name("test_perms");
        let size = 4096;

        let region = ShmRegion::new(&name, size).expect("create");
        let metadata = std::fs::metadata(&region.path).expect("metadata");
        let mode = metadata.permissions().mode() & 0o777;

        assert_eq!(
            mode, 0o600,
            "SHM file should be owner read/write only (0o600), got {:#o}",
            mode
        );
    }

    #[test]
    fn shm_directory_permissions_owner_only() {
        use std::os::unix::fs::PermissionsExt;

        let name = unique_name("test_dir_perms");
        let size = 4096;

        let region = ShmRegion::new(&name, size).expect("create");
        let parent_dir = region.path.parent().expect("parent");
        let metadata = std::fs::metadata(parent_dir).expect("dir metadata");
        let mode = metadata.permissions().mode() & 0o777;

        assert_eq!(
            mode, 0o700,
            "SHM directory should be owner-only (0o700), got {:#o}",
            mode
        );
    }

    #[test]
    fn shm_orphaned_file_reused_by_new_owner() {
        let name = unique_name("test_orphan_reuse");
        let size = 4096;

        let path;
        {
            let region = ShmRegion::new(&name, size).expect("create original");
            path = region.path.clone();
            // Write a pattern
            unsafe {
                let ptr = region.as_ptr() as *mut u8;
                *ptr = 0xDE;
                *ptr.add(1) = 0xAD;
            }
            // Simulate abnormal exit: forget the region to prevent Drop cleanup
            std::mem::forget(region);
        }

        // The SHM file should still exist (no cleanup happened)
        assert!(path.exists(), "Orphaned SHM file should still exist");

        // New process opens the same name — should see it as non-owner
        let region2 = ShmRegion::new(&name, size).expect("reopen orphaned");
        assert!(
            !region2.is_owner(),
            "Should open existing orphaned file as non-owner"
        );

        // Should be able to read the old data (it wasn't cleaned)
        unsafe {
            let ptr = region2.as_ptr();
            assert_eq!(*ptr, 0xDE, "Should see old data from orphaned region");
            assert_eq!(*ptr.add(1), 0xAD);
        }

        // Manual cleanup since we forgot the original owner
        drop(region2); // non-owner drop doesn't clean up
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn shm_region_data_integrity_after_reopen() {
        let name = unique_name("test_reopen_integrity");
        let size = 4096;

        // Write pattern as owner
        let path;
        {
            let region = ShmRegion::new(&name, size).expect("create");
            path = region.path.clone();
            let ptr = region.as_ptr() as *mut u8;
            unsafe {
                for i in 0..size {
                    *ptr.add(i) = (i % 251) as u8; // prime to detect alignment issues
                }
            }
            // Forget to simulate crash (orphan)
            std::mem::forget(region);
        }

        // Reopen and verify all bytes
        let region2 = ShmRegion::new(&name, size).expect("reopen");
        let ptr = region2.as_ptr();
        for i in 0..size {
            let val = unsafe { *ptr.add(i) };
            assert_eq!(
                val,
                (i % 251) as u8,
                "Data mismatch at byte {} after reopen",
                i
            );
        }

        drop(region2);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn shm_multiple_regions_independent() {
        let name_a = unique_name("test_multi_a");
        let name_b = unique_name("test_multi_b");
        let size = 4096;

        let region_a = ShmRegion::new(&name_a, size).expect("create A");
        let region_b = ShmRegion::new(&name_b, size).expect("create B");

        // Write different data to each
        unsafe {
            let pa = region_a.as_ptr() as *mut u8;
            let pb = region_b.as_ptr() as *mut u8;
            *pa = 0xAA;
            *pb = 0xBB;
        }

        // Verify they're independent
        unsafe {
            assert_eq!(*region_a.as_ptr(), 0xAA);
            assert_eq!(*region_b.as_ptr(), 0xBB);
        }

        // Dropping one shouldn't affect the other
        let path_b = region_b.path.clone();
        drop(region_b);
        assert!(!path_b.exists(), "B's file should be cleaned up");
        unsafe {
            assert_eq!(
                *region_a.as_ptr(),
                0xAA,
                "A should still be readable after B dropped"
            );
        }
    }
}

// ============================================================================
// Fallback for other platforms (BSD, etc.) - Use file-based approach
// ============================================================================

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
use crate::memory::platform::shm_topics_dir;
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
use memmap2::{MmapMut, MmapOptions};
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
use std::fs::{File, OpenOptions};

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
impl ShmRegion {
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        // Fallback to /tmp file-based approach
        let horus_shm_dir = PathBuf::from("/tmp/horus/topics");
        std::fs::create_dir_all(&horus_shm_dir)?;

        // Topic names use dot notation (e.g., "motors.cmd_vel") - no conversion needed
        let path = horus_shm_dir.join(format!("horus_{}", name));

        let (file, is_owner) = if path.exists() {
            let file = OpenOptions::new().read(true).write(true).open(&path)?;
            (file, false)
        } else {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(&path)?;
            file.set_len(size as u64)?;
            (file, true)
        };

        // SAFETY: file is a valid open file with sufficient size set above; len(size) matches the file size
        let mut mmap = unsafe { MmapOptions::new().len(size).map_mut(&file)? };
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

    pub fn as_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }
}

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
impl Drop for ShmRegion {
    fn drop(&mut self) {
        if self.owner && self.path.exists() {
            let _ = std::fs::remove_file(&self.path);
        }
    }
}
