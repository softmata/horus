use super::shm_region::ShmRegion;
use crate::error::{HorusError, HorusResult};
use std::marker::PhantomData;
use std::mem;
use std::ops::{Deref, DerefMut};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;

/// Messages >= 4KB use the zero-copy loan/receive path; smaller messages use the fast copy path.
/// This threshold matches SIMD_COPY_THRESHOLD used elsewhere in HORUS.
const ZERO_COPY_THRESHOLD: usize = 4096;

// Safety constants to prevent dangerous configurations
const MAX_CAPACITY: usize = 1_000_000; // Maximum number of elements
const MIN_CAPACITY: usize = 1; // Minimum number of elements
const MAX_ELEMENT_SIZE: usize = 1_000_000; // Maximum size per element in bytes
const MAX_TOTAL_SIZE: usize = 100_000_000; // Maximum total shared memory size (100MB)
const MAX_CONSUMERS: usize = 16; // Maximum number of consumers per topic (MPMC support)

// Magic number to indicate header is fully initialized (prevents race condition)
// This value is written LAST by the owner with Release ordering
// NOTE: V3 indicates 32-bit index optimization (changed from V2 which used 64-bit indices)
// V1: Original layout, V2: CachePadded layout, V3: 32-bit indices for reduced instruction cache pressure
const MAGIC_INITIALIZED: u64 = 0x484F5255535F5633; // "HORUS_V3" in ASCII hex

// Maximum time to wait for initialization (in spin iterations)
const MAX_INIT_WAIT_ITERS: u32 = 1_000_000; // ~100ms on typical hardware

/// Cache-line padded wrapper to prevent false sharing between producer and consumer threads.
///
/// Uses 128-byte alignment for Intel spatial prefetcher compatibility.
/// Intel CPUs prefetch adjacent cache lines in pairs, so using 128 bytes ensures
/// that producer-written fields (head) don't share a prefetch pair with
/// consumer-polled fields (sequence_number).
///
/// Performance impact: Eliminates ~95% of cache invalidation overhead in SPSC scenarios,
/// expected 30-50% latency reduction in producer-consumer hot paths.
#[repr(C, align(128))]
pub struct CachePadded<T> {
    value: T,
    // Implicit padding to 128 bytes due to align(128)
}

impl<T> CachePadded<T> {
    /// Create a new cache-padded value
    #[inline]
    pub const fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T> Deref for CachePadded<T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

impl<T> DerefMut for CachePadded<T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

impl<T: Default> Default for CachePadded<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

/// Header for shared memory ring buffer with false-sharing prevention.
///
/// Memory layout optimized for producer-consumer patterns:
/// - Read-mostly fields (magic, capacity, etc.) are grouped together
/// - Producer-written `head` is on its own 128-byte cache line
/// - `sequence_number` (written by producer, polled by consumers) is on its own cache line
///
/// This layout eliminates false sharing where producer writes to `head` would
/// invalidate consumer cache lines containing `sequence_number`.
///
/// **32-bit Index Optimization (V3)**:
/// Uses AtomicU32 instead of AtomicUsize for indices to reduce instruction cache pressure.
/// On x86-64, 64-bit instructions require a REX prefix (1 extra byte), which increases
/// instruction cache misses in tight loops. 32-bit indices support up to 4 billion entries,
/// far exceeding MAX_CAPACITY (1 million).
#[repr(C, align(128))]
struct RingBufferHeader {
    // === Cache Lines 0-1 (128 bytes): Read-mostly fields ===
    // These are written once during initialization and rarely change thereafter
    magic: AtomicU64,    // 8 bytes - written LAST to signal initialization complete
    capacity: AtomicU32, // 4 bytes - fixed after init (32-bit: max 4B entries)
    tail: AtomicU32,     // 4 bytes - unused, kept for compatibility
    element_size: AtomicUsize, // 8 bytes - fixed after init (stays usize for large elements)
    consumer_count: AtomicU32, // 4 bytes - written occasionally on register/unregister
    _padding0: [u8; 100], // Pad to 128 bytes (8 + 4 + 4 + 8 + 4 = 28, 128 - 28 = 100)

    // === Cache Lines 2-3 (128 bytes): Producer head index ===
    // Written frequently by producer on every publish
    // 32-bit index eliminates REX prefix overhead in hot path
    head: CachePadded<AtomicU32>,

    // === Cache Lines 4-5 (128 bytes): Sequence number ===
    // Written by producer, polled by consumers to detect new data
    // Stays 64-bit to avoid overflow in long-running systems (u32 overflows in ~12h at 100k/s)
    sequence_number: CachePadded<AtomicU64>,
}

/// Lock-free ring buffer in real shared memory using mmap with cache optimization
///
/// **Performance optimization**: Implements rigtorp's local index caching pattern.
/// Instead of loading `head` from shared memory on every read operation,
/// consumers cache the head value locally and only refresh when the buffer
/// appears empty (cached_head == my_tail). This reduces cross-core cache
/// coherency traffic by ~90% in SPSC scenarios, yielding 10-20x throughput
/// improvement in tight polling loops.
///
/// **32-bit Index Optimization (V3)**:
/// Uses u32 instead of usize for indices in local copies, eliminating REX prefix
/// overhead in hot-path operations. Supports up to 4 billion buffer entries.
#[repr(align(64))] // Cache-line aligned structure
pub struct ShmTopic<T> {
    _region: Arc<ShmRegion>,
    header: NonNull<RingBufferHeader>,
    data_ptr: NonNull<u8>,
    capacity: u32,            // 32-bit: max 4B entries (far exceeds MAX_CAPACITY of 1M)
    _consumer_id: u32,        // MPMC: Consumer ID for registration (max 16 consumers)
    consumer_tail: AtomicU32, // MPMC OPTIMIZED: Each consumer tracks tail in LOCAL memory (32-bit)
    /// Rigtorp optimization: Cached head index to avoid reading shared memory on every pop/receive.
    /// Only refreshed when buffer appears empty (cached_head == consumer_tail).
    /// This single optimization provides 10-20x throughput improvement in SPSC scenarios
    /// by eliminating cross-core cache line transfers on the hot path.
    cached_head: AtomicU32, // Relaxed atomic: same perf as Cell on x86/ARM, but sound with Sync
    _phantom: std::marker::PhantomData<T>,
    _padding: [u8; 16], // Padding adjusted for 32-bit fields
}

unsafe impl<T: Send> Send for ShmTopic<T> {}
unsafe impl<T: Send> Sync for ShmTopic<T> {}

/// A loaned sample for zero-copy publishing
/// When dropped, automatically marks the slot as available for consumers
pub struct PublisherSample<'a, T> {
    data_ptr: *mut T,
    _slot_index: usize,
    topic: &'a ShmTopic<T>,
    _phantom: PhantomData<&'a mut T>,
}

/// A received sample for zero-copy consumption
/// When dropped, automatically releases the slot
pub struct ConsumerSample<'a, T> {
    data_ptr: *const T,
    _slot_index: usize,
    _topic: &'a ShmTopic<T>,
    _phantom: PhantomData<&'a T>,
}

unsafe impl<T: Send> Send for PublisherSample<'_, T> {}
unsafe impl<T: Sync> Sync for PublisherSample<'_, T> {}

unsafe impl<T: Send> Send for ConsumerSample<'_, T> {}
unsafe impl<T: Sync> Sync for ConsumerSample<'_, T> {}

impl<T> PublisherSample<'_, T> {
    /// Write data directly into the loaned memory
    pub fn write(&mut self, value: T) {
        // SAFETY: data_ptr points to a valid, properly aligned slot in the mmap'd region,
        // exclusively owned by this PublisherSample (claimed via CAS on head index).
        unsafe {
            std::ptr::write(self.data_ptr, value);
        }
    }
}

impl<T> ConsumerSample<'_, T> {
    /// Get a const reference to the received data
    pub fn get_ref(&self) -> &T {
        // SAFETY: data_ptr points to a valid, initialized T within the mmap'd region;
        // the slot is held by this ConsumerSample for the lifetime of the borrow.
        unsafe { &*self.data_ptr }
    }
}

impl<T> Drop for PublisherSample<'_, T> {
    fn drop(&mut self) {
        // When the publisher sample is dropped, publish it by updating sequence number
        // SAFETY: header pointer is valid for the lifetime of the ShmTopic which outlives this sample.
        let header = unsafe { self.topic.header.as_ref() };
        header.sequence_number.fetch_add(1, Ordering::Release);
    }
}

impl<T> Drop for ConsumerSample<'_, T> {
    fn drop(&mut self) {
        // Consumer sample drop is automatic - just releases the reference
        // The actual slot management is handled by the consumer's tail position
    }
}

impl<T> ShmTopic<T> {
    /// Round up to next power of 2 for optimal modulo performance
    /// Uses bitwise AND instead of expensive division
    #[inline]
    fn next_power_of_2(n: usize) -> usize {
        if n == 0 {
            return 1;
        }
        let mut power = 1;
        while power < n {
            power <<= 1;
        }
        power
    }

    /// Create a new ring buffer in shared memory
    pub fn new(name: &str, capacity: usize) -> HorusResult<Self> {
        // Safety validation: check capacity bounds
        if capacity < MIN_CAPACITY {
            return Err(HorusError::Memory(format!(
                "Capacity {} too small, minimum is {}",
                capacity, MIN_CAPACITY
            )));
        }
        if capacity > MAX_CAPACITY {
            return Err(HorusError::Memory(format!(
                "Capacity {} too large, maximum is {}",
                capacity, MAX_CAPACITY
            )));
        }

        // PERFORMANCE: Round up to power of 2 for bitwise AND optimization
        // This replaces expensive modulo (%) with fast bitwise AND (&)
        let capacity = Self::next_power_of_2(capacity);

        let element_size = mem::size_of::<T>();
        let element_align = mem::align_of::<T>();
        let header_size = mem::size_of::<RingBufferHeader>();

        // Safety validation: check element size
        if element_size == 0 {
            return Err(HorusError::Memory(
                "Cannot create shared memory for zero-sized types".to_string(),
            ));
        }
        if element_size > MAX_ELEMENT_SIZE {
            return Err(HorusError::Memory(format!(
                "Element size {} too large, maximum is {}",
                element_size, MAX_ELEMENT_SIZE
            )));
        }

        // Safety validation: check for overflow in size calculations
        let data_size = capacity
            .checked_mul(element_size)
            .ok_or(HorusError::Memory(
                "Integer overflow calculating data size".to_string(),
            ))?;
        if data_size > MAX_TOTAL_SIZE {
            return Err(HorusError::Memory(format!(
                "Data size {} exceeds maximum {}",
                data_size, MAX_TOTAL_SIZE
            )));
        }

        // Ensure data section is properly aligned
        let aligned_header_size = header_size.div_ceil(element_align) * element_align;
        let total_size = aligned_header_size
            .checked_add(data_size)
            .ok_or(HorusError::Memory(
                "Integer overflow calculating total size".to_string(),
            ))?;

        if total_size > MAX_TOTAL_SIZE {
            return Err(HorusError::Memory(format!(
                "Total size {} exceeds maximum {}",
                total_size, MAX_TOTAL_SIZE
            )));
        }

        // Create shared memory region
        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        // Initialize header with safety checks
        let header_ptr = region.as_ptr() as *mut RingBufferHeader;

        // Safety check: ensure we have enough space for the header
        if region.size() < header_size {
            return Err(HorusError::Memory(
                "Shared memory region too small for header".to_string(),
            ));
        }

        // Safety check: ensure pointer is not null and properly aligned
        if header_ptr.is_null() {
            return Err(HorusError::Memory(
                "Null pointer for shared memory header".to_string(),
            ));
        }
        if !(header_ptr as usize).is_multiple_of(std::mem::align_of::<RingBufferHeader>()) {
            return Err(HorusError::Memory(
                "Header pointer not properly aligned".to_string(),
            ));
        }

        // SAFETY: header_ptr verified non-null and properly aligned above.
        let header = unsafe {
            // This is now safe because we've validated the pointer
            NonNull::new_unchecked(header_ptr)
        };

        // MPMC CRITICAL FIX: Only initialize header if we're the owner (first creator)
        // Otherwise, we would reset consumer_count causing duplicate consumer IDs!
        //
        // 32-bit Index Optimization: capacity stored as u32 for reduced instruction cache pressure
        let actual_capacity: u32 = if is_owner {
            // Validate capacity fits in u32 (should always pass given MAX_CAPACITY = 1M)
            let capacity_u32 = u32::try_from(capacity).map_err(|_| {
                HorusError::Memory(format!(
                    "Capacity {} exceeds u32 maximum (4 billion)",
                    capacity
                ))
            })?;

            // SAFETY: header pointer is valid and aligned (checked above); we are the owner
            // so exclusive initialization access is guaranteed before magic is published.
            unsafe {
                // Initialize all fields BEFORE setting magic (prevents race condition)
                (*header.as_ptr())
                    .capacity
                    .store(capacity_u32, Ordering::Relaxed);
                (*header.as_ptr()).head.store(0, Ordering::Relaxed);
                (*header.as_ptr()).tail.store(0, Ordering::Relaxed);
                (*header.as_ptr())
                    .element_size
                    .store(element_size, Ordering::Relaxed);
                (*header.as_ptr())
                    .consumer_count
                    .store(0, Ordering::Relaxed);
                (*header.as_ptr())
                    .sequence_number
                    .store(0, Ordering::Relaxed);
                // Initialize padding for cache alignment (100 bytes for 128-byte cache line with 32-bit indices)
                (*header.as_ptr())._padding0 = [0; 100];

                // CRITICAL: Write magic number LAST with Release ordering
                // This ensures all previous writes are visible before magic is set
                // Non-owners will spin-wait on this value before reading other fields
                std::sync::atomic::fence(Ordering::Release);
                (*header.as_ptr())
                    .magic
                    .store(MAGIC_INITIALIZED, Ordering::Release);
            }
            capacity_u32
        } else {
            // Not owner - wait for initialization to complete by spinning on magic number
            // This prevents the race condition where we read capacity before it's initialized
            let mut wait_iters = 0u32;
            loop {
                // SAFETY: header pointer is valid and aligned (checked above);
                // atomic load provides synchronization with the owner's Release store.
                let magic = unsafe { (*header.as_ptr()).magic.load(Ordering::Acquire) };
                if magic == MAGIC_INITIALIZED {
                    // Header is fully initialized, safe to read
                    break;
                }
                if magic != 0 && magic != MAGIC_INITIALIZED {
                    // Invalid magic - corrupted or incompatible version
                    return Err(HorusError::Memory(format!(
                        "Topic '{}' has invalid magic number 0x{:X} (corrupted or incompatible version). \
                         Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                        name, magic
                    )));
                }
                // Magic is 0 - owner is still initializing, spin-wait
                wait_iters += 1;
                if wait_iters > MAX_INIT_WAIT_ITERS {
                    return Err(HorusError::Memory(format!(
                        "Topic '{}' initialization timeout: owner process may have crashed during setup. \
                         Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                        name
                    )));
                }
                std::hint::spin_loop();
            }

            // Now safe to read capacity with Acquire ordering (synchronized with owner's Release)
            // Capacity is stored as u32 (32-bit index optimization)
            // SAFETY: header is valid; magic check above ensures initialization is complete.
            let existing_capacity = unsafe { (*header.as_ptr()).capacity.load(Ordering::Acquire) };

            // CRITICAL: Validate existing capacity is power of 2 for bitwise AND optimization
            if !existing_capacity.is_power_of_two() {
                return Err(HorusError::Memory(format!(
                    "Topic '{}' has invalid capacity {} (corrupted shared memory). \
                     Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                    name, existing_capacity
                )));
            }

            // Validate that the existing capacity matches what we calculated
            // Convert capacity (usize) to u32 for comparison with stored value
            let capacity_u32 = u32::try_from(capacity).map_err(|_| {
                HorusError::Memory(format!(
                    "Capacity {} exceeds u32 maximum (4 billion)",
                    capacity
                ))
            })?;
            if existing_capacity != capacity_u32 {
                return Err(HorusError::Memory(format!(
                    "Topic '{}' capacity mismatch: existing={}, requested={} (rounded from original request). \
                     Existing shared memory may be from different session or incompatible version.",
                    name, existing_capacity, capacity_u32
                )));
            }

            existing_capacity
        };

        log::info!(
            "SHM_TRUE: Created true shared memory topic '{}' with capacity: {} (size: {} bytes)",
            name,
            capacity,
            total_size
        );

        // Log topic creation to global log buffer
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        use chrono::Local;
        publish_log(LogEntry {
            timestamp: Local::now().format("%H:%M:%S%.3f").to_string(),
            tick_number: 0, // Topic creation happens outside of tick loop
            node_name: "shm_topic".to_string(),
            log_type: LogType::TopicMap,
            topic: Some(name.to_string()),
            message: format!(
                "Created topic (capacity: {}, size: {} bytes)",
                capacity, total_size
            ),
            tick_us: 0,
            ipc_ns: 0,
        });

        // Data starts after aligned header with comprehensive safety checks
        // SAFETY: pointer arithmetic within bounds of mmap'd region; aligned_header_size
        // is computed to maintain T's alignment. All bounds and alignment are verified below.
        let data_ptr = unsafe {
            let raw_ptr = (region.as_ptr() as *mut u8).add(aligned_header_size);

            // Safety checks for data pointer
            if raw_ptr.is_null() {
                return Err(HorusError::Memory(
                    "Null pointer for data region".to_string(),
                ));
            }

            // Verify we have enough space for the data
            if region.size() < aligned_header_size + data_size {
                return Err(HorusError::Memory(
                    "Shared memory region too small for data".to_string(),
                ));
            }

            // Verify alignment
            if !(raw_ptr as usize).is_multiple_of(element_align) {
                return Err(HorusError::Memory(
                    "Data pointer not properly aligned".to_string(),
                ));
            }

            // Verify the pointer is within the mapped region bounds
            let region_end = (region.as_ptr() as *mut u8).add(region.size());
            let data_end = raw_ptr.add(data_size);
            if data_end > region_end {
                return Err(HorusError::Memory(
                    "Data region extends beyond mapped memory".to_string(),
                ));
            }

            NonNull::new_unchecked(raw_ptr)
        };

        // MPMC FIX: Register this consumer and get a unique ID
        // 32-bit indices: consumer_id and current_head are u32
        // SAFETY: header pointer is valid; atomic operations provide cross-process synchronization.
        let (consumer_id, current_head): (u32, u32) = unsafe {
            let id = (*header.as_ptr())
                .consumer_count
                .fetch_add(1, Ordering::Relaxed);

            // Check if we've exceeded max consumers (MAX_CONSUMERS fits in u32)
            if id as usize >= MAX_CONSUMERS {
                return Err(HorusError::Memory(format!(
                    "Maximum number of consumers ({}) exceeded for topic '{}'",
                    MAX_CONSUMERS, name
                )));
            }

            // MPMC OPTIMIZED: Consumer tail will be initialized in local memory below
            let current_head = (*header.as_ptr()).head.load(Ordering::Relaxed);

            (id, current_head)
        };

        Ok(ShmTopic {
            _region: region,
            header,
            data_ptr,
            capacity: actual_capacity,
            _consumer_id: consumer_id, // MPMC: Consumer ID for registration (u32)
            consumer_tail: AtomicU32::new(current_head), // MPMC OPTIMIZED: Local tail tracking (32-bit)
            cached_head: AtomicU32::new(current_head), // Rigtorp: Cache head locally (32-bit)
            _phantom: std::marker::PhantomData,
            _padding: [0; 16],
        })
    }

    /// Open an existing ring buffer from shared memory
    pub fn open(name: &str) -> HorusResult<Self> {
        let region = Arc::new(ShmRegion::open(name)?);

        // Safety checks for opening existing shared memory
        let header_size = mem::size_of::<RingBufferHeader>();
        if region.size() < header_size {
            return Err(HorusError::Memory(
                "Existing shared memory region too small for header".to_string(),
            ));
        }

        let header_ptr = region.as_ptr() as *mut RingBufferHeader;

        // Safety check: ensure pointer is not null and properly aligned
        if header_ptr.is_null() {
            return Err(HorusError::Memory(
                "Null pointer for existing shared memory header".to_string(),
            ));
        }
        if !(header_ptr as usize).is_multiple_of(std::mem::align_of::<RingBufferHeader>()) {
            return Err(HorusError::Memory(
                "Existing header pointer not properly aligned".to_string(),
            ));
        }

        // SAFETY: header_ptr verified non-null and properly aligned above.
        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        // Wait for initialization to complete by spinning on magic number
        // This prevents reading uninitialized data if owner is still setting up
        let mut wait_iters = 0u32;
        loop {
            // SAFETY: header pointer is valid and aligned (checked above);
            // atomic load provides synchronization with the owner's Release store.
            let magic = unsafe { (*header.as_ptr()).magic.load(Ordering::Acquire) };
            if magic == MAGIC_INITIALIZED {
                // Header is fully initialized, safe to read
                break;
            }
            if magic != 0 && magic != MAGIC_INITIALIZED {
                // Invalid magic - corrupted or incompatible version
                return Err(HorusError::Memory(format!(
                    "Topic '{}' has invalid magic number 0x{:X} (corrupted or incompatible version). \
                     Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                    name, magic
                )));
            }
            // Magic is 0 - owner is still initializing, spin-wait
            wait_iters += 1;
            if wait_iters > MAX_INIT_WAIT_ITERS {
                return Err(HorusError::Memory(format!(
                    "Topic '{}' initialization timeout: owner process may have crashed during setup. \
                     Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                    name
                )));
            }
            std::hint::spin_loop();
        }

        // Now safe to read with Acquire ordering (synchronized with owner's Release)
        // Capacity is stored as u32 (32-bit index optimization)
        // SAFETY: header is valid; magic check above ensures initialization is complete.
        let capacity: u32 = unsafe { (*header.as_ptr()).capacity.load(Ordering::Acquire) };

        // Validate capacity is within safe bounds (compare as usize for constants)
        let capacity_usize = capacity as usize;
        if !(MIN_CAPACITY..=MAX_CAPACITY).contains(&capacity_usize) {
            return Err(HorusError::Memory(format!(
                "Invalid capacity {} in existing shared memory (must be {}-{})",
                capacity, MIN_CAPACITY, MAX_CAPACITY
            )));
        }

        // Validate element size matches
        // SAFETY: header is valid; magic check ensures initialization is complete;
        // Acquire ordering synchronizes with the owner's Release store.
        let stored_element_size =
            unsafe { (*header.as_ptr()).element_size.load(Ordering::Acquire) };
        let expected_element_size = mem::size_of::<T>();
        if stored_element_size != expected_element_size {
            return Err(HorusError::Memory(format!(
                "Element size mismatch: stored {}, expected {}",
                stored_element_size, expected_element_size
            )));
        }

        log::info!(
            "SHM_TRUE: Opened existing shared memory topic '{}' with capacity: {}",
            name,
            capacity
        );

        // Log topic open to global log buffer
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        use chrono::Local;
        publish_log(LogEntry {
            timestamp: Local::now().format("%H:%M:%S%.3f").to_string(),
            tick_number: 0, // Topic open happens outside of tick loop
            node_name: "shm_topic".to_string(),
            log_type: LogType::TopicMap,
            topic: Some(name.to_string()),
            message: format!("Opened existing topic (capacity: {})", capacity),
            tick_us: 0,
            ipc_ns: 0,
        });

        let element_align = mem::align_of::<T>();
        let header_size = mem::size_of::<RingBufferHeader>();
        let aligned_header_size = header_size.div_ceil(element_align) * element_align;

        // SAFETY: pointer arithmetic within bounds of mmap'd region; aligned_header_size
        // maintains T's alignment. All bounds and alignment are verified below.
        let data_ptr = unsafe {
            let raw_ptr = (region.as_ptr() as *mut u8).add(aligned_header_size);

            // Safety checks for data pointer in existing shared memory
            if raw_ptr.is_null() {
                return Err(HorusError::Memory(
                    "Null pointer for existing data region".to_string(),
                ));
            }

            // Calculate expected total size (use usize for size calculations)
            let expected_data_size = capacity_usize * expected_element_size;
            let expected_total_size = aligned_header_size + expected_data_size;

            // Verify we have enough space for the data
            if region.size() < expected_total_size {
                return Err(HorusError::Memory(format!(
                    "Existing shared memory too small: {} < {}",
                    region.size(),
                    expected_total_size
                )));
            }

            // Verify alignment
            if !(raw_ptr as usize).is_multiple_of(element_align) {
                return Err(HorusError::Memory(
                    "Existing data pointer not properly aligned".to_string(),
                ));
            }

            // Verify the pointer is within the mapped region bounds
            let region_end = (region.as_ptr() as *mut u8).add(region.size());
            let data_end = raw_ptr.add(expected_data_size);
            if data_end > region_end {
                return Err(HorusError::Memory(
                    "Existing data region extends beyond mapped memory".to_string(),
                ));
            }

            NonNull::new_unchecked(raw_ptr)
        };

        // MPMC FIX: Register as a new consumer and get current head position to start from
        // 32-bit indices: consumer_id and current_head are u32
        // SAFETY: header pointer is valid; atomic operations provide cross-process synchronization.
        let (consumer_id, current_head): (u32, u32) = unsafe {
            let id = (*header.as_ptr())
                .consumer_count
                .fetch_add(1, Ordering::Relaxed);

            // Check if we've exceeded max consumers (MAX_CONSUMERS fits in u32)
            if id as usize >= MAX_CONSUMERS {
                return Err(HorusError::Memory(format!(
                    "Maximum number of consumers ({}) exceeded for topic '{}'",
                    MAX_CONSUMERS, name
                )));
            }

            let head = (*header.as_ptr()).head.load(Ordering::Relaxed);

            // MPMC OPTIMIZED: Consumer tail will be initialized in local memory below

            (id, head)
        };

        Ok(ShmTopic {
            _region: region,
            header,
            data_ptr,
            capacity,
            _consumer_id: consumer_id, // MPMC: Consumer ID for registration (u32)
            consumer_tail: AtomicU32::new(current_head), // MPMC OPTIMIZED: Local tail tracking (32-bit)
            cached_head: AtomicU32::new(current_head), // Rigtorp: Cache head locally (32-bit)
            _phantom: std::marker::PhantomData,
            _padding: [0; 16],
        })
    }

    /// **ZERO-OVERHEAD Push**: Ultra-fast push with NO BOUNDS CHECKING.
    ///
    /// This is the fastest possible push for trusted, high-performance scenarios.
    /// Skips ALL safety checks for maximum throughput.
    ///
    /// # Performance
    /// - Target: <100ns for MPMC shared memory
    /// - Single CAS attempt, immediate return on contention
    /// - No bounds checking (trusts ring buffer math)
    /// - No eprintln! calls
    /// - Minimal sequence tracking
    ///
    /// # Safety
    /// This method assumes:
    /// - Capacity is a power of 2
    /// - Ring buffer indices are always valid due to bitwise AND modulo
    /// - No corruption in shared memory header
    ///
    /// # Returns
    /// - `Ok(())` if push succeeded
    /// - `Err(msg)` if CAS failed (contention) - caller should retry or backoff
    #[inline(always)]
    pub(crate) fn push_fast(&self, msg: T) -> Result<(), T> {
        // SAFETY: header pointer is valid for the lifetime of self; initialized in constructor.
        let header = unsafe { self.header.as_ref() };

        let head = header.head.load(Ordering::Relaxed);
        // Bitwise AND modulo - capacity must be power of 2
        let next = (head + 1) & (self.capacity - 1);

        // Single CAS attempt - return immediately on contention
        match header
            .head
            .compare_exchange(head, next, Ordering::Release, Ordering::Relaxed)
        {
            Ok(_) => {
                // Write directly - NO BOUNDS CHECKING
                // SAFETY: head is always < capacity due to bitwise AND modulo, so
                // byte_offset is within bounds of the mmap'd data region. slot_ptr is
                // properly aligned for T (guaranteed by aligned_header_size calculation).
                unsafe {
                    let byte_offset = (head as usize) * mem::size_of::<T>();
                    let slot_ptr = self.data_ptr.as_ptr().add(byte_offset) as *mut T;
                    std::ptr::write(slot_ptr, msg);
                }
                // NOTE: No sequence_number update - consumers use cached_head
                // This saves one atomic operation (~5-10ns on x86_64)
                Ok(())
            }
            Err(_) => Err(msg),
        }
    }

    /// **ZERO-OVERHEAD Pop**: Ultra-fast pop with NO BOUNDS CHECKING.
    ///
    /// This is the fastest possible pop for trusted, high-performance scenarios.
    /// Uses Rigtorp optimization (cached head) and skips all safety checks.
    ///
    /// # Performance
    /// - Target: <100ns for MPMC shared memory
    /// - Cached head eliminates ~90% of cross-core cache transfers
    /// - No bounds checking (trusts ring buffer math)
    /// - No eprintln! calls
    ///
    /// # Safety
    /// This method assumes:
    /// - Capacity is a power of 2
    /// - Ring buffer indices are always valid due to bitwise AND modulo
    /// - No corruption in shared memory header
    ///
    /// # Returns
    /// - `Some(T)` if data was available
    /// - `None` if buffer is empty
    #[inline(always)]
    pub(crate) fn pop_fast(&self) -> Option<T>
    where
        T: Clone,
    {
        // SAFETY: header pointer is valid for the lifetime of self; initialized in constructor.
        let header = unsafe { self.header.as_ref() };

        // Get this consumer's current tail position from LOCAL MEMORY
        let my_tail = self.consumer_tail.load(Ordering::Relaxed);

        // RIGTORP OPTIMIZATION: Use cached head first
        let mut cached = self.cached_head.load(Ordering::Relaxed);

        if my_tail == cached {
            // Buffer appears empty - refresh from shared memory
            cached = header.head.load(Ordering::Acquire);
            self.cached_head.store(cached, Ordering::Relaxed);

            if my_tail == cached {
                return None; // Buffer is actually empty
            }
        }

        // Calculate next position - bitwise AND modulo
        let next_tail = (my_tail + 1) & (self.capacity - 1);

        // Update this consumer's tail position in LOCAL MEMORY
        self.consumer_tail.store(next_tail, Ordering::Relaxed);

        // Read directly - NO BOUNDS CHECKING
        // SAFETY: my_tail is always < capacity due to bitwise AND modulo, so byte_offset
        // is within bounds of the mmap'd data region. slot_ptr is properly aligned for T.
        let msg = unsafe {
            let byte_offset = (my_tail as usize) * mem::size_of::<T>();
            let slot_ptr = self.data_ptr.as_ptr().add(byte_offset) as *const T;
            (*slot_ptr).clone()
        };

        Some(msg)
    }

    /// Check if the buffer is empty from this consumer's perspective.
    ///
    /// # Performance
    /// This method refreshes the cached head from shared memory to provide
    /// an accurate answer. Use sparingly in tight loops - the `pop()` method
    /// already handles empty detection efficiently.
    ///
    /// # Returns
    /// `true` if no messages are available for this consumer, `false` otherwise.
    #[inline]
    pub fn is_empty(&self) -> bool {
        // SAFETY: header pointer is valid for the lifetime of self; initialized in constructor.
        let header = unsafe { self.header.as_ref() };
        let my_tail = self.consumer_tail.load(Ordering::Relaxed);

        // Refresh cached head from shared memory for accurate answer
        let head = header.head.load(Ordering::Acquire);
        self.cached_head.store(head, Ordering::Relaxed);

        my_tail == head
    }

    /// Loan a slot in the shared memory for zero-copy publishing
    /// Returns a PublisherSample that provides direct access to shared memory
    pub(crate) fn loan(&self) -> crate::error::HorusResult<PublisherSample<'_, T>> {
        // SAFETY: header pointer is valid for the lifetime of self; initialized in constructor.
        let header = unsafe { self.header.as_ref() };

        loop {
            let head = header.head.load(Ordering::Relaxed);
            // PERFORMANCE: Use bitwise AND instead of modulo (capacity is power of 2)
            let next = (head + 1) & (self.capacity - 1);

            // Try to claim this slot atomically
            // Note: Buffer full checking removed (was buggy - sequence_number increments forever)
            match header.head.compare_exchange_weak(
                head,
                next,
                Ordering::Acquire, // Synchronize with consumers
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    // Successfully claimed slot, return sample pointing to it
                    // SAFETY: head index is bounds-checked below; byte offset is within the
                    // mmap'd data region. Pointer is properly aligned for T. Slot is exclusively
                    // owned via successful CAS on head.
                    unsafe {
                        // Bounds checking
                        if head >= self.capacity {
                            eprintln!(
                                "Critical safety violation: head index {} >= capacity {}",
                                head, self.capacity
                            );
                            return Err(HorusError::Memory(format!(
                                "Head index {} >= capacity {}",
                                head, self.capacity
                            )));
                        }

                        // Cast u32 index to usize for byte offset calculation
                        let byte_offset = (head as usize) * mem::size_of::<T>();
                        let data_ptr = self.data_ptr.as_ptr().add(byte_offset) as *mut T;

                        // Prefetch the data slot we're about to write to (reduces write latency)
                        #[cfg(target_arch = "x86_64")]
                        {
                            use std::arch::x86_64::{_mm_prefetch, _MM_HINT_T0};
                            _mm_prefetch(data_ptr as *const i8, _MM_HINT_T0);
                        }

                        // Verify bounds
                        let data_region_size = (self.capacity as usize) * mem::size_of::<T>();
                        if byte_offset + mem::size_of::<T>() > data_region_size {
                            eprintln!(
                                "Critical safety violation: loan would exceed data region bounds"
                            );
                            return Err(HorusError::Memory(
                                "Loan would exceed data region bounds".to_string(),
                            ));
                        }

                        return Ok(PublisherSample {
                            data_ptr,
                            _slot_index: head as usize,
                            topic: self,
                            _phantom: PhantomData,
                        });
                    }
                }
                Err(_) => {
                    // Another thread updated head, retry
                    continue;
                }
            }
        }
    }

    /// Receive a message using zero-copy access
    /// Returns a ConsumerSample that provides direct access to shared memory
    ///
    /// **Rigtorp Optimization**: Uses cached head index to avoid reading shared memory
    /// on every call. The head is only refreshed when the buffer appears empty
    /// (my_tail == cached_head). This reduces atomic loads by ~90% in polling loops.
    pub fn receive(&self) -> Option<ConsumerSample<'_, T>> {
        // SAFETY: header pointer is valid for the lifetime of self; initialized in constructor.
        let header = unsafe { self.header.as_ref() };

        // MPMC OPTIMIZED: Get this consumer's current tail position from LOCAL MEMORY
        let my_tail = self.consumer_tail.load(Ordering::Relaxed);

        // EARLY PREFETCH: Start loading the slot we're likely to read BEFORE checking
        // if there's data available. This hides memory latency behind the Rigtorp check.
        // Even if there's no data, the prefetch cost is negligible.
        #[cfg(target_arch = "x86_64")]
        {
            use std::arch::x86_64::{_mm_prefetch, _MM_HINT_T0};
            // SAFETY: pointer arithmetic on data_ptr; prefetch is a hint and is
            // safe even if the address is not yet valid (no-op on x86 for bad addresses).
            let slot_offset = (my_tail as usize) * mem::size_of::<T>();
            let slot_addr = unsafe { self.data_ptr.as_ptr().add(slot_offset) };
            unsafe {
                _mm_prefetch(slot_addr as *const i8, _MM_HINT_T0);
            }
        }

        // RIGTORP OPTIMIZATION: Use cached head first, only refresh from shared memory
        // when buffer appears empty. This eliminates ~90% of cross-core cache transfers.
        let mut cached = self.cached_head.load(Ordering::Relaxed);

        if my_tail == cached {
            // Buffer appears empty - refresh from shared memory
            cached = header.head.load(Ordering::Acquire);
            self.cached_head.store(cached, Ordering::Relaxed);

            if my_tail == cached {
                // Buffer is actually empty
                return None;
            }
        }

        // Validate positions
        if my_tail >= self.capacity {
            eprintln!(
                "Critical safety violation: consumer tail {} >= capacity {}",
                my_tail, self.capacity
            );
            return None;
        }

        if cached >= self.capacity {
            eprintln!(
                "Critical safety violation: head {} >= capacity {}",
                cached, self.capacity
            );
            return None;
        }

        // Calculate next position for this consumer and update in local memory
        // PERFORMANCE: Use bitwise AND instead of modulo (capacity is power of 2)
        let next_tail = (my_tail + 1) & (self.capacity - 1);
        self.consumer_tail.store(next_tail, Ordering::Relaxed);

        // Return sample pointing to the message in shared memory
        // SAFETY: my_tail is bounds-checked above (< capacity); byte_offset is within the
        // mmap'd data region. Pointer is properly aligned for T. Bounds verified below.
        unsafe {
            // Cast u32 index to usize for byte offset calculation
            let byte_offset = (my_tail as usize) * mem::size_of::<T>();
            let data_ptr = self.data_ptr.as_ptr().add(byte_offset) as *const T;

            // NOTE: Prefetch was moved to the top of this function (EARLY PREFETCH)
            // to hide memory latency behind the Rigtorp check. Data should already
            // be in L1 cache by the time we reach this point.

            // Verify bounds
            let data_region_size = (self.capacity as usize) * mem::size_of::<T>();
            if byte_offset + mem::size_of::<T>() > data_region_size {
                eprintln!("Critical safety violation: receive would exceed data region bounds");
                return None;
            }

            Some(ConsumerSample {
                data_ptr,
                _slot_index: my_tail as usize,
                _topic: self,
                _phantom: PhantomData,
            })
        }
    }

    /// Read the most recent message without advancing consumer position
    /// Unlike receive(), this always returns the latest message if one exists,
    /// regardless of when this consumer started reading.
    /// This is useful for reading static/infrequently-updated data.
    pub fn read_latest(&self) -> Option<ConsumerSample<'_, T>> {
        // SAFETY: header pointer is valid for the lifetime of self; initialized in constructor.
        let header = unsafe { self.header.as_ref() };
        let current_head = header.head.load(Ordering::Acquire);

        // If head is 0, no messages have been written yet
        if current_head == 0 {
            return None;
        }

        // The most recent message is at (head - 1) in a ring buffer
        // Handle wrap-around for ring buffer (already checked head != 0 above)
        let latest_slot = current_head - 1;

        // Validate position
        if latest_slot >= self.capacity {
            eprintln!(
                "Critical safety violation: latest_slot {} >= capacity {}",
                latest_slot, self.capacity
            );
            return None;
        }

        // Return sample pointing to the most recent message
        // Note: This does NOT advance consumer_tail, so the same message
        // will be returned on subsequent calls until a new message is written
        // SAFETY: latest_slot is bounds-checked above (< capacity); byte_offset is within the
        // mmap'd data region. Pointer is properly aligned for T. Bounds verified below.
        unsafe {
            // Cast u32 index to usize for byte offset calculation
            let byte_offset = (latest_slot as usize) * mem::size_of::<T>();
            let data_ptr = self.data_ptr.as_ptr().add(byte_offset) as *const T;

            // Verify bounds
            let data_region_size = (self.capacity as usize) * mem::size_of::<T>();
            if byte_offset + mem::size_of::<T>() > data_region_size {
                eprintln!("Critical safety violation: read_latest would exceed data region bounds");
                return None;
            }

            Some(ConsumerSample {
                data_ptr,
                _slot_index: latest_slot as usize,
                _topic: self,
                _phantom: PhantomData,
            })
        }
    }

    /// Loan a slot and immediately write data (convenience method)
    /// This is equivalent to loan() followed by write(), but more convenient
    pub(crate) fn loan_and_write(&self, value: T) -> Result<(), T> {
        match self.loan() {
            Ok(mut sample) => {
                sample.write(value);
                // Sample is automatically published when dropped
                Ok(())
            }
            Err(_) => Err(value),
        }
    }

    /// Smart send: auto-selects the optimal publish strategy based on message size.
    ///
    /// - Messages < 4KB use `push_fast()` (copy path, no bounds checking overhead)
    /// - Messages >= 4KB use `loan_and_write()` (zero-copy path via shared memory loan)
    ///
    /// This is the recommended publish API for ShmTopic.
    #[inline]
    pub fn send(&self, msg: T) -> Result<(), T> {
        if mem::size_of::<T>() >= ZERO_COPY_THRESHOLD {
            self.loan_and_write(msg)
        } else {
            self.push_fast(msg)
        }
    }

    /// Smart recv: auto-selects the optimal consume strategy based on message size.
    ///
    /// - Messages < 4KB use `pop_fast()` (copy path, cached head optimization)
    /// - Messages >= 4KB use `receive()` (zero-copy read, then clone)
    ///
    /// This is the recommended consume API for ShmTopic.
    #[inline]
    pub fn recv(&self) -> Option<T>
    where
        T: Clone,
    {
        if mem::size_of::<T>() >= ZERO_COPY_THRESHOLD {
            self.receive().map(|sample| sample.get_ref().clone())
        } else {
            self.pop_fast()
        }
    }
}

impl<T> Drop for ShmTopic<T> {
    fn drop(&mut self) {
        // MPMC FIX: Decrement consumer count when this consumer is dropped
        // This prevents the "Maximum number of consumers exceeded" error
        // when consumers exit without proper cleanup
        // SAFETY: header pointer is valid for the lifetime of self; atomic operation
        // provides cross-process synchronization for consumer count tracking.
        let prev_count = unsafe {
            (*self.header.as_ptr())
                .consumer_count
                .fetch_sub(1, Ordering::AcqRel)
        };

        // If we were the last consumer (prev_count was 1), clean up the topic file
        // This ensures stale topic files don't accumulate
        if prev_count == 1 {
            self._region.force_cleanup();
        }
    }
}
