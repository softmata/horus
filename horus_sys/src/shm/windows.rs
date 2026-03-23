// Windows shared memory: CreateFileMappingW with pagefile backing
//
// Uses INVALID_HANDLE_VALUE for pure shared memory (no temp files).

use anyhow::Result;

/// Cross-platform shared memory region for high-performance IPC.
///
/// Windows backend: `CreateFileMappingW` with pagefile backing.
#[derive(Debug)]
pub struct ShmRegion {
    ptr: *mut u8,
    handle: *mut std::ffi::c_void, // HANDLE
    topic_name: String,
    size: usize,
    owner: bool,
}

impl ShmRegion {
    /// Create or open a shared memory region using Windows API (pagefile-backed).
    pub fn new(name: &str, size: usize) -> Result<Self> {
        anyhow::ensure!(size > 0, "SHM region size must be > 0");
        anyhow::ensure!(!name.is_empty(), "SHM region name must not be empty");
        use windows_sys::Win32::Foundation::{
            CloseHandle, GetLastError, ERROR_ALREADY_EXISTS, INVALID_HANDLE_VALUE,
        };
        use windows_sys::Win32::System::Memory::{
            CreateFileMappingW, MapViewOfFile, FILE_MAP_ALL_ACCESS, PAGE_READWRITE,
        };

        let mapping_name = format!("Local\\horus_{}", name);

        let wide_name: Vec<u16> = mapping_name
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();

        // SAFETY: INVALID_HANDLE_VALUE creates pagefile-backed mapping; wide_name is valid
        let handle = unsafe {
            CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                std::ptr::null(),
                PAGE_READWRITE,
                (size >> 32) as u32,
                size as u32,
                wide_name.as_ptr(),
            )
        };
        // Capture error code immediately — GetLastError is not sticky across API calls
        let last_error = unsafe { GetLastError() };

        if handle.is_null() {
            anyhow::bail!("CreateFileMappingW failed: error {}", last_error);
        }

        let is_owner = last_error != ERROR_ALREADY_EXISTS;

        // SAFETY: handle is valid (non-null checked above)
        let view = unsafe { MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size) };

        let ptr = view.Value as *mut u8;
        if ptr.is_null() {
            unsafe { CloseHandle(handle) };
            // SAFETY: GetLastError is always safe
            anyhow::bail!("MapViewOfFile failed: error {}", unsafe { GetLastError() });
        }

        if is_owner {
            // SAFETY: ptr is valid from MapViewOfFile, size matches
            unsafe {
                std::ptr::write_bytes(ptr, 0, size);
            }
            // Write topic metadata file so discovery can find this SHM region
            let _ = super::write_topic_meta(name, size);
        }

        Ok(Self {
            ptr,
            handle,
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
        // SAFETY: ptr is valid from MapViewOfFile, size bytes are mapped
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }

    /// View the mapped memory as a mutable byte slice.
    #[inline]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        // SAFETY: ptr is valid, &mut self ensures exclusive access
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

    /// No filesystem backing path on Windows (uses pagefile).
    #[inline]
    pub fn backing_path(&self) -> Option<&std::path::Path> {
        None
    }

    /// Grow the region to `new_size` bytes without synchronization.
    ///
    /// On Windows, file mappings cannot be resized in place. This creates a new
    /// mapping with the larger size and copies data from the old mapping.
    ///
    /// # Safety
    ///
    /// The caller must ensure no other thread is concurrently reading from or
    /// writing to this memory region via raw pointers derived from `as_ptr()`.
    pub unsafe fn grow_unchecked(&mut self, new_size: usize) -> Result<()> {
        use windows_sys::Win32::Foundation::{CloseHandle, INVALID_HANDLE_VALUE};
        use windows_sys::Win32::System::Memory::{
            CreateFileMappingW, MapViewOfFile, UnmapViewOfFile, FILE_MAP_ALL_ACCESS,
            MEMORY_MAPPED_VIEW_ADDRESS, PAGE_READWRITE,
        };

        anyhow::ensure!(
            new_size > self.size,
            "grow_unchecked: new_size ({}) must be > current size ({})",
            new_size,
            self.size
        );

        // Create a new file mapping with the larger size
        let name: Vec<u16> = self
            .topic_name
            .encode_utf16()
            .chain(std::iter::once(0))
            .collect();
        let new_handle = CreateFileMappingW(
            INVALID_HANDLE_VALUE,
            std::ptr::null(),
            PAGE_READWRITE,
            (new_size >> 32) as u32,
            new_size as u32,
            name.as_ptr(),
        );
        anyhow::ensure!(
            !new_handle.is_null(),
            "CreateFileMappingW for grow failed: {}",
            std::io::Error::last_os_error()
        );

        let new_ptr = MapViewOfFile(new_handle, FILE_MAP_ALL_ACCESS, 0, 0, new_size);
        anyhow::ensure!(
            !new_ptr.Value.is_null(),
            "MapViewOfFile for grow failed: {}",
            std::io::Error::last_os_error()
        );

        // Copy old data to new mapping
        std::ptr::copy_nonoverlapping(self.ptr, new_ptr.Value as *mut u8, self.size);

        // Unmap and close old resources
        let old_view = MEMORY_MAPPED_VIEW_ADDRESS {
            Value: self.ptr as *mut std::ffi::c_void,
        };
        UnmapViewOfFile(old_view);
        CloseHandle(self.handle);

        self.ptr = new_ptr.Value as *mut u8;
        self.handle = new_handle;
        self.size = new_size;
        Ok(())
    }
}

impl Drop for ShmRegion {
    fn drop(&mut self) {
        use windows_sys::Win32::Foundation::CloseHandle;
        use windows_sys::Win32::System::Memory::UnmapViewOfFile;

        if self.owner {
            super::remove_topic_meta(&self.topic_name);
        }

        // SAFETY: self.ptr is a valid mapped view; self.handle is a valid file mapping handle
        unsafe {
            let view = windows_sys::Win32::System::Memory::MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self.ptr as *mut std::ffi::c_void,
            };
            UnmapViewOfFile(view);
            CloseHandle(self.handle);
        }
        // Windows automatically cleans up named file mappings when all handles are closed
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}
