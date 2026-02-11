// HORUS Shared Memory Region - Cross-platform optimized shared memory
//
// Each platform uses its optimal shared memory mechanism:
// - Linux: /dev/shm files (tmpfs - RAM-backed, already optimal)
// - macOS: shm_open() + mmap (POSIX shared memory, RAM-backed via Mach)
// - Windows: CreateFileMappingW with INVALID_HANDLE_VALUE (pagefile-backed, optimized for IPC)

use crate::error::{HorusError, HorusResult};
use std::path::PathBuf;

#[cfg(target_os = "linux")]
use crate::memory::platform::shm_topics_dir;
#[cfg(target_os = "linux")]
use memmap2::{MmapMut, MmapOptions};
#[cfg(target_os = "linux")]
use std::fs::{File, OpenOptions};

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
        std::fs::create_dir_all(&horus_shm_dir)?;

        // Topic names use dot notation (e.g., "motors.cmd_vel") - no conversion needed
        // Names can also contain "/" for namespacing (e.g., "links/sensor_test")
        let path = horus_shm_dir.join(format!("horus_{}", name));

        // Create parent directory if the name contains "/" (e.g., "links/sensor_test")
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let (file, is_owner) = if path.exists() {
            let file = OpenOptions::new().read(true).write(true).open(&path)?;
            let metadata = file.metadata()?;
            if metadata.len() < size as u64 {
                file.set_len(size as u64)?;
            }
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

    /// Open existing shared memory region (no creation)
    pub fn open(name: &str) -> HorusResult<Self> {
        // Use flat namespace - all topics in same directory
        let horus_shm_dir = shm_topics_dir();
        // Topic names use dot notation (e.g., "motors.cmd_vel") - no conversion needed
        let path = horus_shm_dir.join(format!("horus_{}", name));

        if !path.exists() {
            return Err(HorusError::Memory(format!(
                "Shared memory '{}' does not exist",
                name
            )));
        }

        let file = OpenOptions::new().read(true).write(true).open(&path)?;
        let metadata = file.metadata()?;
        let size = metadata.len() as usize;
        // SAFETY: file is a valid open file; len(size) matches the actual file size from metadata
        let mmap = unsafe { MmapOptions::new().len(size).map_mut(&file)? };

        Ok(Self {
            mmap,
            size,
            path,
            _file: file,

            owner: false,
        })
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.mmap.as_mut_ptr()
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// Force cleanup of the shared memory file (for use when last consumer exits)
    /// This bypasses the owner check and removes the file unconditionally
    pub fn force_cleanup(&self) {
        if self.path.exists() {
            let _ = std::fs::remove_file(&self.path);
        }
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

#[cfg(target_os = "macos")]
impl ShmRegion {
    /// Create or open a shared memory region using shm_open (RAM-backed)
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        use std::ffi::CString;

        // Use flat namespace - all topics share same prefix (ROS-like simplicity)
        let shm_name = format!("/horus_{}", name);
        let c_name = CString::new(shm_name.clone()).map_err(|e| {
            HorusError::Memory(format!(
                "Invalid shm name '{}': topic names cannot contain null bytes: {}",
                shm_name, e
            ))
        })?;

        // Try to open existing first
        // SAFETY: c_name is a valid null-terminated CString; flags are valid POSIX constants
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666) };

        let (fd, is_owner) = if fd >= 0 {
            // Opened existing
            (fd, false)
        } else {
            // Create new
            // SAFETY: c_name is a valid null-terminated CString; O_CREAT|O_RDWR|O_EXCL are valid POSIX flags
            let fd = unsafe {
                libc::shm_open(
                    c_name.as_ptr(),
                    libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                    0o666,
                )
            };
            if fd < 0 {
                // Race condition: someone else created it, try opening again
                // SAFETY: c_name is still a valid null-terminated CString
                let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666) };
                if fd < 0 {
                    return Err(HorusError::Memory(format!(
                        "Failed to open/create shm '{}': {}",
                        shm_name,
                        std::io::Error::last_os_error()
                    )));
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
                        "Failed to set shm '{}' to {} bytes: {} (check /dev/shm space with 'df -h /dev/shm')",
                        shm_name, size, std::io::Error::last_os_error()
                    )));
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
            return Err(HorusError::Memory(format!(
                "Failed to mmap shm: {}",
                std::io::Error::last_os_error()
            )));
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

    /// Open existing shared memory region
    pub fn open(name: &str) -> HorusResult<Self> {
        use std::ffi::CString;

        // Use flat namespace - all topics share same prefix
        let shm_name = format!("/horus_{}", name);
        let c_name = CString::new(shm_name.clone()).map_err(|e| {
            HorusError::Memory(format!(
                "Invalid shm name '{}': topic names cannot contain null bytes: {}",
                shm_name, e
            ))
        })?;

        // SAFETY: c_name is a valid null-terminated CString; O_RDWR is a valid POSIX flag
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0o666) };

        if fd < 0 {
            return Err(HorusError::Memory(format!(
                "Shared memory '{}' does not exist",
                name
            )));
        }

        // Get size
        // SAFETY: libc::stat is a C struct where all-zeros is a valid representation
        let mut stat: libc::stat = unsafe { std::mem::zeroed() };
        // SAFETY: fd is a valid open file descriptor; stat is a valid mutable reference
        if unsafe { libc::fstat(fd, &mut stat) } != 0 {
            // SAFETY: fd is a valid open file descriptor
            unsafe { libc::close(fd) };
            return Err(HorusError::Memory(format!(
                "Failed to stat shm: {}",
                std::io::Error::last_os_error()
            )));
        }
        let size = stat.st_size as usize;

        // SAFETY: fd is valid from shm_open, size > 0 from fstat, flags are valid POSIX mmap constants
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
            return Err(HorusError::Memory(format!(
                "Failed to mmap shm: {}",
                std::io::Error::last_os_error()
            )));
        }

        Ok(Self {
            ptr: ptr as *mut u8,
            fd,
            shm_name,
            size,

            owner: false,
        })
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// Force cleanup of the shared memory (for use when last consumer exits)
    /// This bypasses the owner check and removes the shm unconditionally
    pub fn force_cleanup(&self) {
        if let Ok(c_name) = std::ffi::CString::new(self.shm_name.clone()) {
            // SAFETY: c_name is a valid null-terminated CString
            unsafe { libc::shm_unlink(c_name.as_ptr()) };
        }
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
            return Err(HorusError::Memory(format!(
                "CreateFileMappingW failed: error {}",
                // SAFETY: GetLastError is always safe to call after a Windows API failure
                unsafe { GetLastError() }
            )));
        }

        // SAFETY: GetLastError is always safe to call; checks if mapping already existed
        let is_owner = unsafe { GetLastError() } != ERROR_ALREADY_EXISTS;

        // Map view of file
        // SAFETY: handle is a valid file mapping from CreateFileMappingW (non-zero checked above)
        let ptr = unsafe { MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size) };

        if ptr.is_null() {
            // SAFETY: handle is a valid file mapping handle from CreateFileMappingW
            unsafe { CloseHandle(handle) };
            return Err(HorusError::Memory(format!(
                "MapViewOfFile failed: error {}",
                // SAFETY: GetLastError is always safe to call after a Windows API failure
                unsafe { GetLastError() }
            )));
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

    /// Open existing shared memory region
    pub fn open(name: &str) -> HorusResult<Self> {
        use windows_sys::Win32::Foundation::{CloseHandle, GetLastError};
        use windows_sys::Win32::System::Memory::{
            MapViewOfFile, OpenFileMappingW, FILE_MAP_ALL_ACCESS,
        };

        // Use flat namespace - all topics share same prefix
        let mapping_name = format!("Local\\horus_{}", name);
        let wide_name: Vec<u16> = mapping_name
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();

        // SAFETY: wide_name is a valid null-terminated wide string; FILE_MAP_ALL_ACCESS is a valid flag
        let handle = unsafe {
            OpenFileMappingW(
                FILE_MAP_ALL_ACCESS,
                0, // bInheritHandle = FALSE
                wide_name.as_ptr(),
            )
        };

        if handle == 0 {
            return Err(HorusError::Memory(format!(
                "Shared memory '{}' does not exist",
                name
            )));
        }

        // Map view - we don't know size, map entire region
        // SAFETY: handle is a valid file mapping from OpenFileMappingW (non-zero checked above)
        let ptr = unsafe {
            MapViewOfFile(
                handle,
                FILE_MAP_ALL_ACCESS,
                0,
                0,
                0, // Map entire file
            )
        };

        if ptr.is_null() {
            // SAFETY: handle is a valid file mapping handle from OpenFileMappingW
            unsafe { CloseHandle(handle) };
            return Err(HorusError::Memory(format!(
                "MapViewOfFile failed: error {}",
                // SAFETY: GetLastError is always safe to call after a Windows API failure
                unsafe { GetLastError() }
            )));
        }

        // Note: We can't easily get the size of an existing mapping on Windows
        // without additional tracking. For now, use a reasonable default.
        // In practice, the caller should know the expected size.
        let size = 0; // Unknown - caller should track

        Ok(Self {
            ptr: ptr as *mut u8,
            handle,
            size,

            owner: false,
        })
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// Force cleanup of the shared memory (for use when last consumer exits)
    /// On Windows, cleanup is automatic when all handles are closed, so this is a no-op
    pub fn force_cleanup(&self) {
        // Windows automatically cleans up named file mappings when all handles are closed
        // No explicit cleanup needed
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

// Thread safety - shared memory regions can be sent between threads
// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state;
// concurrent access is managed by atomic operations at the topic/transport layer
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

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

    pub fn open(name: &str) -> HorusResult<Self> {
        // Topic names use dot notation - no conversion needed
        let path = PathBuf::from("/tmp/horus/topics").join(format!("horus_{}", name));
        if !path.exists() {
            return Err(HorusError::Memory(format!(
                "Shared memory '{}' does not exist",
                name
            )));
        }
        let file = OpenOptions::new().read(true).write(true).open(&path)?;
        let size = file.metadata()?.len() as usize;
        // SAFETY: file is a valid open file; len(size) matches the actual file size from metadata
        let mmap = unsafe { MmapOptions::new().len(size).map_mut(&file)? };
        Ok(Self {
            mmap,
            size,
            path,
            _file: file,

            owner: false,
        })
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.mmap.as_mut_ptr()
    }
    pub fn size(&self) -> usize {
        self.size
    }
    pub fn is_owner(&self) -> bool {
        self.owner
    }

    /// Force cleanup of the shared memory file (for use when last consumer exits)
    pub fn force_cleanup(&self) {
        if self.path.exists() {
            let _ = std::fs::remove_file(&self.path);
        }
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
