// Windows shared memory: CreateFileMappingW with pagefile backing
//
// Uses INVALID_HANDLE_VALUE for pure shared memory (no temp files).

use anyhow::Result;

/// Kernel object name for a topic's file mapping, as a NUL-terminated UTF-16
/// string.
///
/// `new()`, `open_existing()` and `grow_unchecked()` must agree on this string
/// or they address different kernel objects. `grow_unchecked` built its name
/// from the bare topic name, omitting the `Local\horus_` prefix, so it created
/// an unrelated private section and silently forked the topic: the grower
/// published into the new object while every other process stayed attached to
/// the real one, frozen at the moment of the grow with no error on either side.
/// Defining the name once is what stops the three call sites drifting again.
fn mapping_name_wide(name: &str) -> Vec<u16> {
    format!("Local\\horus_{}", name)
        .encode_utf16()
        .chain(std::iter::once(0))
        .collect()
}

/// Cross-platform shared memory region for high-performance IPC.
///
/// Windows backend: `CreateFileMappingW` with pagefile backing.
#[derive(Debug)]
pub struct ShmRegion {
    ptr: *mut u8,
    /// `(view, handle)` of every mapping `grow_unchecked` replaced, kept alive
    /// deliberately. See the Linux backend's `retired` field: `Topic` shares one
    /// `Arc<ShmRegion>` across clones, so unmapping the old view on grow leaves
    /// every clone's cached pointers dangling.
    ///
    /// Caveat specific to Windows: a named section cannot be resized in place,
    /// so `grow_unchecked` *copies* into a brand-new section. Retention stops
    /// the crash, but unlike the POSIX backends the old view is a stale SNAPSHOT
    /// rather than a coherent second view of the same pages -- a reader that has
    /// not yet re-synced sees pre-grow data. That incoherence is inherent to the
    /// copy and predates this change; it is why Windows auto-grow is documented
    /// as unsupported for concurrent readers.
    retired: Vec<(*mut u8, *mut std::ffi::c_void)>,
    handle: *mut std::ffi::c_void, // HANDLE
    topic_name: String,
    size: usize,
    owner: bool,
}

/// Size in bytes of the section behind `handle`, or `None` if it cannot be
/// measured.
///
/// Maps with length 0, which requests the WHOLE section rather than a caller-
/// chosen window — that is what avoids the "view larger than section" mismatch
/// this function exists to diagnose. It is not a guarantee of success: the call
/// can still fail (insufficient rights, for one), and `None` is returned then.
///
/// Mapped `FILE_MAP_READ`, not `FILE_MAP_ALL_ACCESS`. `VirtualQuery` only reads
/// the mapping's metadata, and asking for write access means the measurement
/// fails wherever the caller has read but not write — which is precisely a case
/// where the informative "capacity mismatch" message is worth having and the
/// bare "error 5" is not.
///
/// # Safety
///
/// `handle` must be a valid file-mapping handle.
unsafe fn measure_section(handle: *mut std::ffi::c_void) -> Option<usize> {
    use windows_sys::Win32::System::Memory::{
        MapViewOfFile, UnmapViewOfFile, VirtualQuery, FILE_MAP_READ, MEMORY_BASIC_INFORMATION,
    };

    let view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, 0);
    let ptr = view.Value as *mut u8;
    if ptr.is_null() {
        return None;
    }
    let mut info: MEMORY_BASIC_INFORMATION = std::mem::zeroed();
    let written = VirtualQuery(
        ptr as *const std::ffi::c_void,
        &mut info,
        std::mem::size_of::<MEMORY_BASIC_INFORMATION>(),
    );
    let measured = if written == 0 {
        None
    } else {
        Some(info.RegionSize)
    };
    // Unmap the address `MapViewOfFile` returned, rather than rebuilding one
    // from `ptr`. The reconstruction happened to be identical, but it is an
    // assumption about the struct's layout that nothing here needs to make.
    UnmapViewOfFile(view);
    measured
}

impl ShmRegion {
    /// Open an existing region without creating one when the publisher is absent.
    pub fn open_existing(name: &str, minimum_size: usize) -> Result<Self> {
        anyhow::ensure!(minimum_size > 0, "SHM region size must be > 0");
        super::validate_region_name(name)?;
        use windows_sys::Win32::Foundation::{CloseHandle, GetLastError};
        use windows_sys::Win32::System::Memory::{
            MapViewOfFile, OpenFileMappingW, UnmapViewOfFile, VirtualQuery, FILE_MAP_ALL_ACCESS,
            MEMORY_BASIC_INFORMATION, MEMORY_MAPPED_VIEW_ADDRESS,
        };
        // FILE_MAP_ALL_ACCESS, not FILE_MAP_READ. `as_slice_mut()` promises a
        // writable slice on every backend and Linux honours it; a read-only view
        // here turned the first store into an access violation, so safe code
        // that works on Linux crashed on Windows.
        let wide_name = mapping_name_wide(name);
        let handle = unsafe { OpenFileMappingW(FILE_MAP_ALL_ACCESS, 0, wide_name.as_ptr()) };
        if handle.is_null() {
            anyhow::bail!("OpenFileMappingW failed: error {}", unsafe {
                GetLastError()
            });
        }
        // Map the whole section (length 0) rather than just `minimum_size`, so
        // `len()`/`as_slice()` describe the region the creator actually made —
        // the macOS and Linux backends take their size from fstat/metadata, and
        // storing `minimum_size` here made the same region report different
        // lengths depending on the platform.
        let view = unsafe { MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, 0) };
        let ptr = view.Value as *mut u8;
        if ptr.is_null() {
            let err = unsafe { GetLastError() };
            unsafe { CloseHandle(handle) };
            anyhow::bail!("MapViewOfFile failed: error {}", err);
        }
        // VirtualQuery reports the extent of the mapped view; fall back to the
        // caller's minimum if the query fails rather than claiming a size we did
        // not measure.
        let size = unsafe {
            let mut info: MEMORY_BASIC_INFORMATION = std::mem::zeroed();
            let written = VirtualQuery(
                ptr as *const std::ffi::c_void,
                &mut info,
                std::mem::size_of::<MEMORY_BASIC_INFORMATION>(),
            );
            if written == 0 {
                minimum_size
            } else {
                info.RegionSize
            }
        };
        if size < minimum_size {
            unsafe {
                UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                    Value: ptr as *mut std::ffi::c_void,
                });
                CloseHandle(handle);
            }
            anyhow::bail!("existing SHM region is too small");
        }
        Ok(Self {
            ptr,
            retired: Vec::new(),
            handle,
            topic_name: name.to_string(),
            size,
            owner: false,
        })
    }

    /// Create or open a shared memory region using Windows API (pagefile-backed).
    pub fn new(name: &str, size: usize) -> Result<Self> {
        anyhow::ensure!(size > 0, "SHM region size must be > 0");
        super::validate_region_name(name)?;
        use windows_sys::Win32::Foundation::{
            CloseHandle, GetLastError, ERROR_ALREADY_EXISTS, INVALID_HANDLE_VALUE,
        };
        use windows_sys::Win32::System::Memory::{
            CreateFileMappingW, MapViewOfFile, FILE_MAP_ALL_ACCESS, PAGE_READWRITE,
        };

        let wide_name = mapping_name_wide(name);

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
            // SAFETY: GetLastError is always safe
            let map_err = unsafe { GetLastError() };

            // The common cause is invisible in the error code. A named Windows
            // section cannot be resized (see `grow_unchecked`), so when the
            // section already existed and the caller asks to map more than the
            // creator made, MapViewOfFile refuses with ERROR_ACCESS_DENIED (5).
            // "error 5" tells the reader nothing about a capacity mismatch, and
            // the same code covers genuine permission problems. Measure the
            // section and say which one it was.
            if !is_owner {
                // SAFETY: handle is a valid section handle, checked non-null above.
                let existing = unsafe { measure_section(handle) };
                unsafe { CloseHandle(handle) };
                if let Some(actual) = existing.filter(|&a| a < size) {
                    anyhow::bail!(
                        "shared memory region '{name}' already exists at {actual} bytes, \
                         but this handle asked to map {size}. A named Windows section \
                         cannot be resized, so the mapping was refused \
                         (MapViewOfFile error {map_err}). Open the region with the same \
                         capacity its creator used. Linux and macOS tolerate this \
                         mismatch, so code that works there can fail only here."
                    );
                }
                anyhow::bail!(
                    "MapViewOfFile failed for existing region '{name}' ({size} bytes \
                     requested): error {map_err}"
                );
            }
            unsafe { CloseHandle(handle) };
            anyhow::bail!("MapViewOfFile failed: error {map_err}");
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
            retired: Vec::new(),
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
    /// # Limitation
    ///
    /// A named Windows section cannot be resized, so this call is expected to
    /// FAIL on Windows: it re-opens `Local\horus_<topic>` — the topic's real
    /// kernel object — and `MapViewOfFile` refuses a view larger than the
    /// existing section. Failing loudly is the point. Building the name from the
    /// bare topic name instead created a private, unrelated section and reported
    /// success, which forked the topic: the grower published into memory nobody
    /// else was mapped to, and every subscriber sat on a sequence counter that
    /// would never advance again.
    ///
    /// Real Windows growth needs a generation counter published in the topic
    /// header (`Local\horus_<topic>_g<n>`), with other processes re-opening the
    /// new generation by name when they observe the migration epoch change.
    /// Until that exists, Windows auto-grow is unsupported.
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

        use windows_sys::Win32::Foundation::{
            CloseHandle, GetLastError, ERROR_ALREADY_EXISTS, INVALID_HANDLE_VALUE,
        };
        use windows_sys::Win32::System::Memory::{
            CreateFileMappingW, MapViewOfFile, FILE_MAP_ALL_ACCESS, PAGE_READWRITE,
        };

        anyhow::ensure!(
            new_size > self.size,
            "grow_unchecked: new_size ({}) must be > current size ({})",
            new_size,
            self.size
        );

        // Same name every other call site uses — see `mapping_name_wide`.
        let name = mapping_name_wide(&self.topic_name);
        let new_handle = CreateFileMappingW(
            INVALID_HANDLE_VALUE,
            std::ptr::null(),
            PAGE_READWRITE,
            (new_size >> 32) as u32,
            new_size as u32,
            name.as_ptr(),
        );
        // Capture immediately: GetLastError is not sticky across API calls.
        let last_error = GetLastError();

        anyhow::ensure!(
            !new_handle.is_null(),
            "CreateFileMappingW for grow failed: {}",
            std::io::Error::last_os_error()
        );

        // A named section that already exists comes back AT ITS ORIGINAL SIZE --
        // the size arguments above are ignored. Growing is therefore impossible,
        // and continuing is worse than failing: `new_ptr` would be a second view
        // of the SAME physical pages, so the copy below would memcpy the region
        // onto itself. On a live ring that is silent corruption -- any concurrent
        // write landing between the copy's load and store of a chunk is rolled
        // back to its pre-copy value, and a rolled-back `sequence_or_head`
        // re-issues sequence numbers that were already published.
        //
        // It only reached the copy at all when the requested size still fit the
        // section's page-rounded extent, so `MapViewOfFile` succeeded and the
        // "cannot be resized" error below never fired. That made the failure
        // size-dependent and rare: Windows CI lost 2 messages in 1800.
        //
        // `new()` has always read ERROR_ALREADY_EXISTS to decide ownership; this
        // path simply never did.
        if last_error == ERROR_ALREADY_EXISTS {
            CloseHandle(new_handle);
            anyhow::bail!(
                "cannot grow '{}' from {} to {} bytes: a named Windows section \
                 already exists and cannot be resized in place. Windows auto-grow \
                 is unsupported; growing here would map the same pages twice and \
                 memcpy the live region onto itself.",
                self.topic_name,
                self.size,
                new_size
            );
        }

        let new_ptr = MapViewOfFile(new_handle, FILE_MAP_ALL_ACCESS, 0, 0, new_size);
        if new_ptr.Value.is_null() {
            // Capture the error before CloseHandle, which clobbers it. Without
            // this close, every failed grow leaked a kernel handle and the
            // pagefile-backed section it keeps alive.
            let err = std::io::Error::last_os_error();
            CloseHandle(new_handle);
            anyhow::bail!(
                "MapViewOfFile for grow failed: {}. A named Windows section \
                 cannot be resized in place; Windows auto-grow is unsupported.",
                err
            );
        }

        // Copy old data to new mapping
        std::ptr::copy_nonoverlapping(self.ptr, new_ptr.Value as *mut u8, self.size);

        // Retain the old view and its handle rather than releasing them here:
        // see the `retired` field. Released together in `Drop`.
        self.retired.push((self.ptr, self.handle));

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
            // Views and sections retained across grows (see `retired`).
            for (ptr, handle) in self.retired.drain(..) {
                let old = windows_sys::Win32::System::Memory::MEMORY_MAPPED_VIEW_ADDRESS {
                    Value: ptr as *mut std::ffi::c_void,
                };
                UnmapViewOfFile(old);
                CloseHandle(handle);
            }
        }
        // Windows automatically cleans up named file mappings when all handles are closed
    }
}

// SAFETY: ShmRegion uses OS-level shared memory with no thread-local state
unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

#[cfg(test)]
mod tests {
    use super::*;

    /// A capacity mismatch must say so, not just "error 5".
    ///
    /// This is the failure a subscriber hits when it opens a topic with a
    /// different capacity than the publisher created it with. It is invisible on
    /// Linux and macOS, so the only place the message can be wrong is the only
    /// place it is ever read.
    #[test]
    fn reopening_a_section_larger_than_it_is_explains_itself() {
        let name = format!("shm_size_mismatch_{}", std::process::id());

        let _creator = ShmRegion::new(&name, 4096).expect("create the small section");

        let err = ShmRegion::new(&name, 1 << 20)
            .expect_err("mapping 1 MiB over a 4 KiB section must fail on Windows");
        let msg = err.to_string();

        assert!(
            msg.contains("already exists at"),
            "the error must name the mismatch, got: {msg}"
        );
        assert!(
            msg.contains("cannot be resized"),
            "the error must say why it is unfixable by retrying, got: {msg}"
        );
        assert!(
            msg.contains(&name),
            "the error must name the region, got: {msg}"
        );
        assert!(
            msg.contains("1048576"),
            "the error must state the size that was asked for, got: {msg}"
        );
    }

    /// The improved branch must not swallow the ordinary create path: a fresh
    /// region of a sane size still works.
    #[test]
    fn creating_a_fresh_section_still_succeeds() {
        let name = format!("shm_fresh_{}", std::process::id());
        let region = ShmRegion::new(&name, 4096).expect("fresh create");
        assert!(region.len() >= 4096, "got {}", region.len());
    }
}
