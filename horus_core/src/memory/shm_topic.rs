use super::shm_region::ShmRegion;
use crate::error::HorusResult;
use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;

// Safety constants to prevent dangerous configurations
const MAX_CAPACITY: usize = 1_000_000; // Maximum number of elements
const MIN_CAPACITY: usize = 1; // Minimum number of elements
const MAX_ELEMENT_SIZE: usize = 1_000_000; // Maximum size per element in bytes
const MAX_TOTAL_SIZE: usize = 100_000_000; // Maximum total shared memory size (100MB)
const MAX_CONSUMERS: usize = 16; // Maximum number of consumers per topic (MPMC support)

// Magic number to indicate header is fully initialized (prevents race condition)
// This value is written LAST by the owner with Release ordering
const MAGIC_INITIALIZED: u64 = 0x484F5255535F4F4B; // "HORUS_OK" in ASCII hex

// Maximum time to wait for initialization (in spin iterations)
const MAX_INIT_WAIT_ITERS: u32 = 1_000_000; // ~100ms on typical hardware

/// Header for shared memory ring buffer with cache-line alignment
#[repr(C, align(64))] // Cache-line aligned for optimal performance (x86_64 cache line = 64 bytes)
struct RingBufferHeader {
    magic: AtomicU64, // Magic number - written LAST to signal initialization complete
    capacity: AtomicUsize,
    head: AtomicUsize,
    tail: AtomicUsize, // This is now unused - kept for compatibility
    element_size: AtomicUsize,
    consumer_count: AtomicUsize,
    sequence_number: AtomicUsize, // Global sequence counter
    _padding: [u8; 8],            // Pad to 64-byte cache line boundary (1*8 + 6*8 + 8 = 64)
}

/// Lock-free ring buffer in real shared memory using mmap with cache optimization
#[repr(align(64))] // Cache-line aligned structure
pub struct ShmTopic<T> {
    _region: Arc<ShmRegion>,
    header: NonNull<RingBufferHeader>,
    data_ptr: NonNull<u8>,
    capacity: usize,
    _consumer_id: usize, // MPMC: Consumer ID for registration (not used for tail tracking)
    consumer_tail: AtomicUsize, // MPMC OPTIMIZED: Each consumer tracks tail in LOCAL memory (not shared)
    _phantom: std::marker::PhantomData<T>,
    _padding: [u8; 16], // Pad to prevent false sharing
}

unsafe impl<T: Send> Send for ShmTopic<T> {}
unsafe impl<T: Send> Sync for ShmTopic<T> {}

/// A loaned sample for zero-copy publishing
/// When dropped, automatically marks the slot as available for consumers
pub struct PublisherSample<'a, T> {
    data_ptr: *mut T,
    #[allow(dead_code)]
    slot_index: usize,
    #[allow(dead_code)]
    topic: &'a ShmTopic<T>,
    _phantom: PhantomData<&'a mut T>,
}

/// A received sample for zero-copy consumption  
/// When dropped, automatically releases the slot
pub struct ConsumerSample<'a, T> {
    data_ptr: *const T,
    #[allow(dead_code)]
    slot_index: usize,
    #[allow(dead_code)]
    topic: &'a ShmTopic<T>,
    _phantom: PhantomData<&'a T>,
}

unsafe impl<T: Send> Send for PublisherSample<'_, T> {}
unsafe impl<T: Sync> Sync for PublisherSample<'_, T> {}

unsafe impl<T: Send> Send for ConsumerSample<'_, T> {}
unsafe impl<T: Sync> Sync for ConsumerSample<'_, T> {}

impl<T> PublisherSample<'_, T> {
    /// Get a mutable reference to the loaned memory
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.data_ptr
    }

    /// Write data directly into the loaned memory
    pub fn write(&mut self, value: T) {
        unsafe {
            std::ptr::write(self.data_ptr, value);
        }
    }

    /// Get a mutable reference to the data (unsafe because it bypasses borrow checker)
    ///
    /// # Safety
    ///
    /// The caller must ensure that no other references to this data exist,
    /// and that the data pointer is valid and properly aligned.
    pub unsafe fn as_mut(&mut self) -> &mut T {
        &mut *self.data_ptr
    }
}

impl<T> ConsumerSample<'_, T> {
    /// Get a const reference to the received data
    pub fn get_ref(&self) -> &T {
        unsafe { &*self.data_ptr }
    }

    /// Get the raw pointer to the data
    pub fn as_ptr(&self) -> *const T {
        self.data_ptr
    }

    /// Read the data by copy (for types that implement Copy)
    pub fn read(&self) -> T
    where
        T: Copy,
    {
        unsafe { std::ptr::read(self.data_ptr) }
    }
}

impl<T> Drop for PublisherSample<'_, T> {
    fn drop(&mut self) {
        // When the publisher sample is dropped, publish it by updating sequence number
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
            return Err(format!(
                "Capacity {} too small, minimum is {}",
                capacity, MIN_CAPACITY
            )
            .into());
        }
        if capacity > MAX_CAPACITY {
            return Err(format!(
                "Capacity {} too large, maximum is {}",
                capacity, MAX_CAPACITY
            )
            .into());
        }

        // PERFORMANCE: Round up to power of 2 for bitwise AND optimization
        // This replaces expensive modulo (%) with fast bitwise AND (&)
        let capacity = Self::next_power_of_2(capacity);

        let element_size = mem::size_of::<T>();
        let element_align = mem::align_of::<T>();
        let header_size = mem::size_of::<RingBufferHeader>();

        // Safety validation: check element size
        if element_size == 0 {
            return Err("Cannot create shared memory for zero-sized types".into());
        }
        if element_size > MAX_ELEMENT_SIZE {
            return Err(format!(
                "Element size {} too large, maximum is {}",
                element_size, MAX_ELEMENT_SIZE
            )
            .into());
        }

        // Safety validation: check for overflow in size calculations
        let data_size = capacity
            .checked_mul(element_size)
            .ok_or("Integer overflow calculating data size")?;
        if data_size > MAX_TOTAL_SIZE {
            return Err(
                format!("Data size {} exceeds maximum {}", data_size, MAX_TOTAL_SIZE).into(),
            );
        }

        // Ensure data section is properly aligned
        let aligned_header_size = header_size.div_ceil(element_align) * element_align;
        let total_size = aligned_header_size
            .checked_add(data_size)
            .ok_or("Integer overflow calculating total size")?;

        if total_size > MAX_TOTAL_SIZE {
            return Err(format!(
                "Total size {} exceeds maximum {}",
                total_size, MAX_TOTAL_SIZE
            )
            .into());
        }

        // Create shared memory region
        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        // Initialize header with safety checks
        let header_ptr = region.as_ptr() as *mut RingBufferHeader;

        // Safety check: ensure we have enough space for the header
        if region.size() < header_size {
            return Err("Shared memory region too small for header".into());
        }

        // Safety check: ensure pointer is not null and properly aligned
        if header_ptr.is_null() {
            return Err("Null pointer for shared memory header".into());
        }
        if (header_ptr as usize) % std::mem::align_of::<RingBufferHeader>() != 0 {
            return Err("Header pointer not properly aligned".into());
        }

        let header = unsafe {
            // This is now safe because we've validated the pointer
            NonNull::new_unchecked(header_ptr)
        };

        // MPMC CRITICAL FIX: Only initialize header if we're the owner (first creator)
        // Otherwise, we would reset consumer_count causing duplicate consumer IDs!
        let actual_capacity = if is_owner {
            unsafe {
                // Initialize all fields BEFORE setting magic (prevents race condition)
                (*header.as_ptr())
                    .capacity
                    .store(capacity, Ordering::Relaxed);
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
                // MPMC OPTIMIZED: Consumer tails now tracked in local memory (not in header)
                (*header.as_ptr())._padding = [0; 8]; // Initialize padding for cache alignment

                // CRITICAL: Write magic number LAST with Release ordering
                // This ensures all previous writes are visible before magic is set
                // Non-owners will spin-wait on this value before reading other fields
                std::sync::atomic::fence(Ordering::Release);
                (*header.as_ptr())
                    .magic
                    .store(MAGIC_INITIALIZED, Ordering::Release);
            }
            capacity
        } else {
            // Not owner - wait for initialization to complete by spinning on magic number
            // This prevents the race condition where we read capacity before it's initialized
            let mut wait_iters = 0u32;
            loop {
                let magic = unsafe { (*header.as_ptr()).magic.load(Ordering::Acquire) };
                if magic == MAGIC_INITIALIZED {
                    // Header is fully initialized, safe to read
                    break;
                }
                if magic != 0 && magic != MAGIC_INITIALIZED {
                    // Invalid magic - corrupted or incompatible version
                    return Err(format!(
                        "Topic '{}' has invalid magic number 0x{:X} (corrupted or incompatible version). \
                         Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                        name, magic
                    )
                    .into());
                }
                // Magic is 0 - owner is still initializing, spin-wait
                wait_iters += 1;
                if wait_iters > MAX_INIT_WAIT_ITERS {
                    return Err(format!(
                        "Topic '{}' initialization timeout: owner process may have crashed during setup. \
                         Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                        name
                    )
                    .into());
                }
                std::hint::spin_loop();
            }

            // Now safe to read capacity with Acquire ordering (synchronized with owner's Release)
            let existing_capacity = unsafe { (*header.as_ptr()).capacity.load(Ordering::Acquire) };

            // CRITICAL: Validate existing capacity is power of 2 for bitwise AND optimization
            if !existing_capacity.is_power_of_two() {
                return Err(format!(
                    "Topic '{}' has invalid capacity {} (corrupted shared memory). \
                     Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                    name, existing_capacity
                )
                .into());
            }

            // Validate that the existing capacity matches what we calculated
            if existing_capacity != capacity {
                return Err(format!(
                    "Topic '{}' capacity mismatch: existing={}, requested={} (rounded from original request). \
                     Existing shared memory may be from different session or incompatible version.",
                    name, existing_capacity, capacity
                )
                .into());
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
        let data_ptr = unsafe {
            let raw_ptr = (region.as_ptr() as *mut u8).add(aligned_header_size);

            // Safety checks for data pointer
            if raw_ptr.is_null() {
                return Err("Null pointer for data region".into());
            }

            // Verify we have enough space for the data
            if region.size() < aligned_header_size + data_size {
                return Err("Shared memory region too small for data".into());
            }

            // Verify alignment
            if (raw_ptr as usize) % element_align != 0 {
                return Err("Data pointer not properly aligned".into());
            }

            // Verify the pointer is within the mapped region bounds
            let region_end = (region.as_ptr() as *mut u8).add(region.size());
            let data_end = raw_ptr.add(data_size);
            if data_end > region_end {
                return Err("Data region extends beyond mapped memory".into());
            }

            NonNull::new_unchecked(raw_ptr)
        };

        // MPMC FIX: Register this consumer and get a unique ID
        let (consumer_id, current_head) = unsafe {
            let id = (*header.as_ptr())
                .consumer_count
                .fetch_add(1, Ordering::Relaxed);

            // Check if we've exceeded max consumers
            if id >= MAX_CONSUMERS {
                return Err(format!(
                    "Maximum number of consumers ({}) exceeded for topic '{}'",
                    MAX_CONSUMERS, name
                )
                .into());
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
            _consumer_id: consumer_id, // MPMC: Consumer ID for registration
            consumer_tail: AtomicUsize::new(current_head), // MPMC OPTIMIZED: Local tail tracking
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
            return Err("Existing shared memory region too small for header".into());
        }

        let header_ptr = region.as_ptr() as *mut RingBufferHeader;

        // Safety check: ensure pointer is not null and properly aligned
        if header_ptr.is_null() {
            return Err("Null pointer for existing shared memory header".into());
        }
        if (header_ptr as usize) % std::mem::align_of::<RingBufferHeader>() != 0 {
            return Err("Existing header pointer not properly aligned".into());
        }

        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        // Wait for initialization to complete by spinning on magic number
        // This prevents reading uninitialized data if owner is still setting up
        let mut wait_iters = 0u32;
        loop {
            let magic = unsafe { (*header.as_ptr()).magic.load(Ordering::Acquire) };
            if magic == MAGIC_INITIALIZED {
                // Header is fully initialized, safe to read
                break;
            }
            if magic != 0 && magic != MAGIC_INITIALIZED {
                // Invalid magic - corrupted or incompatible version
                return Err(format!(
                    "Topic '{}' has invalid magic number 0x{:X} (corrupted or incompatible version). \
                     Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                    name, magic
                )
                .into());
            }
            // Magic is 0 - owner is still initializing, spin-wait
            wait_iters += 1;
            if wait_iters > MAX_INIT_WAIT_ITERS {
                return Err(format!(
                    "Topic '{}' initialization timeout: owner process may have crashed during setup. \
                     Please delete shared memory files in /dev/shm/horus/topics/ and restart.",
                    name
                )
                .into());
            }
            std::hint::spin_loop();
        }

        // Now safe to read with Acquire ordering (synchronized with owner's Release)
        let capacity = unsafe { (*header.as_ptr()).capacity.load(Ordering::Acquire) };

        // Validate capacity is within safe bounds
        if !(MIN_CAPACITY..=MAX_CAPACITY).contains(&capacity) {
            return Err(format!(
                "Invalid capacity {} in existing shared memory (must be {}-{})",
                capacity, MIN_CAPACITY, MAX_CAPACITY
            )
            .into());
        }

        // Validate element size matches
        let stored_element_size =
            unsafe { (*header.as_ptr()).element_size.load(Ordering::Acquire) };
        let expected_element_size = mem::size_of::<T>();
        if stored_element_size != expected_element_size {
            return Err(format!(
                "Element size mismatch: stored {}, expected {}",
                stored_element_size, expected_element_size
            )
            .into());
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

        let data_ptr = unsafe {
            let raw_ptr = (region.as_ptr() as *mut u8).add(aligned_header_size);

            // Safety checks for data pointer in existing shared memory
            if raw_ptr.is_null() {
                return Err("Null pointer for existing data region".into());
            }

            // Calculate expected total size
            let expected_data_size = capacity * expected_element_size;
            let expected_total_size = aligned_header_size + expected_data_size;

            // Verify we have enough space for the data
            if region.size() < expected_total_size {
                return Err(format!(
                    "Existing shared memory too small: {} < {}",
                    region.size(),
                    expected_total_size
                )
                .into());
            }

            // Verify alignment
            if (raw_ptr as usize) % element_align != 0 {
                return Err("Existing data pointer not properly aligned".into());
            }

            // Verify the pointer is within the mapped region bounds
            let region_end = (region.as_ptr() as *mut u8).add(region.size());
            let data_end = raw_ptr.add(expected_data_size);
            if data_end > region_end {
                return Err("Existing data region extends beyond mapped memory".into());
            }

            NonNull::new_unchecked(raw_ptr)
        };

        // MPMC FIX: Register as a new consumer and get current head position to start from
        let (consumer_id, current_head) = unsafe {
            let id = (*header.as_ptr())
                .consumer_count
                .fetch_add(1, Ordering::Relaxed);

            // Check if we've exceeded max consumers
            if id >= MAX_CONSUMERS {
                return Err(format!(
                    "Maximum number of consumers ({}) exceeded for topic '{}'",
                    MAX_CONSUMERS, name
                )
                .into());
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
            _consumer_id: consumer_id, // MPMC: Consumer ID for registration
            consumer_tail: AtomicUsize::new(current_head), // MPMC OPTIMIZED: Local tail tracking
            _phantom: std::marker::PhantomData,
            _padding: [0; 16],
        })
    }

    /// Push a message; returns Err(msg) if the buffer is full
    /// Thread-safe for multiple producers
    /// Uses sequence numbering instead of tail checking for multi-consumer safety
    pub fn push(&self, msg: T) -> Result<(), T> {
        let header = unsafe { self.header.as_ref() };

        loop {
            let head = header.head.load(Ordering::Relaxed);
            // PERFORMANCE: Use bitwise AND instead of modulo (capacity is power of 2)
            let next = (head + 1) & (self.capacity - 1);

            // For multi-consumer, we need to check if buffer would wrap around
            // and potentially overwrite unread messages. For now, use a simple
            // heuristic: don't fill more than 75% of buffer capacity
            let current_sequence = header.sequence_number.load(Ordering::Relaxed);
            let max_unread = (self.capacity * 3) / 4; // Allow 75% fill

            if current_sequence >= max_unread
                && current_sequence - header.sequence_number.load(Ordering::Relaxed) >= max_unread
            {
                // Buffer getting too full for safe multi-consumer operation
                return Err(msg);
            }

            // Try to claim this slot atomically
            match header.head.compare_exchange_weak(
                head,
                next,
                Ordering::Release,
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    // Successfully claimed slot, now write data with comprehensive bounds checking
                    unsafe {
                        // Comprehensive bounds checking
                        if head >= self.capacity {
                            // This should never happen due to modulo arithmetic, but be extra safe
                            eprintln!(
                                "Critical safety violation: head index {} >= capacity {}",
                                head, self.capacity
                            );
                            return Err(msg);
                        }

                        // Calculate byte offset and verify it's within bounds
                        let byte_offset = head * mem::size_of::<T>();
                        let slot_ptr = self.data_ptr.as_ptr().add(byte_offset) as *mut T;

                        // Verify the write location is within our data region
                        let data_region_size = self.capacity * mem::size_of::<T>();
                        if byte_offset + mem::size_of::<T>() > data_region_size {
                            eprintln!(
                                "Critical safety violation: write would exceed data region bounds"
                            );
                            return Err(msg);
                        }

                        // Safe to write now that we've verified bounds
                        std::ptr::write(slot_ptr, msg);
                    }

                    // Increment global sequence number
                    header.sequence_number.fetch_add(1, Ordering::Relaxed);
                    return Ok(());
                }
                Err(_) => {
                    // Another thread updated head, retry
                    continue;
                }
            }
        }
    }

    /// Pop a message; returns None if the buffer is empty
    /// MPMC FIX: Thread-safe for multiple consumers - each consumer tracks position in shared memory
    pub fn pop(&self) -> Option<T>
    where
        T: Clone,
    {
        let header = unsafe { self.header.as_ref() };

        // MPMC OPTIMIZED: Get this consumer's current tail position from LOCAL MEMORY
        let my_tail = self.consumer_tail.load(Ordering::Relaxed);
        let current_head = header.head.load(Ordering::Acquire); // Synchronize with producer's Release

        // Validate tail position is within bounds
        if my_tail >= self.capacity {
            eprintln!(
                "Critical safety violation: consumer tail {} >= capacity {}",
                my_tail, self.capacity
            );
            return None;
        }

        // Validate head position is within bounds
        if current_head >= self.capacity {
            eprintln!(
                "Critical safety violation: head {} >= capacity {}",
                current_head, self.capacity
            );
            return None;
        }

        if my_tail == current_head {
            // No new messages for this consumer
            return None;
        }

        // Calculate next position for this consumer
        // PERFORMANCE: Use bitwise AND instead of modulo (capacity is power of 2)
        let next_tail = (my_tail + 1) & (self.capacity - 1);

        // MPMC OPTIMIZED: Update this consumer's tail position in LOCAL MEMORY
        self.consumer_tail.store(next_tail, Ordering::Relaxed);

        // Read the message (non-destructive - message stays for other consumers)
        // Bounds already validated above
        let msg = unsafe {
            // Calculate byte offset and verify it's within bounds
            let byte_offset = my_tail * mem::size_of::<T>();
            let slot_ptr = self.data_ptr.as_ptr().add(byte_offset) as *const T;

            // Verify the read location is within our data region
            let data_region_size = self.capacity * mem::size_of::<T>();
            if byte_offset + mem::size_of::<T>() > data_region_size {
                eprintln!("Critical safety violation: read would exceed data region bounds");
                return None;
            }

            // MPMC CRITICAL FIX: Clone instead of read (move) to avoid double-free
            // Multiple consumers must be able to read the same slot
            (*slot_ptr).clone()
        };

        Some(msg)
    }

    /// Loan a slot in the shared memory for zero-copy publishing
    /// Returns a PublisherSample that provides direct access to shared memory
    pub fn loan(&self) -> crate::error::HorusResult<PublisherSample<'_, T>> {
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
                    unsafe {
                        // Bounds checking
                        if head >= self.capacity {
                            eprintln!(
                                "Critical safety violation: head index {} >= capacity {}",
                                head, self.capacity
                            );
                            return Err(format!(
                                "Head index {} >= capacity {}",
                                head, self.capacity
                            )
                            .into());
                        }

                        let byte_offset = head * mem::size_of::<T>();
                        let data_ptr = self.data_ptr.as_ptr().add(byte_offset) as *mut T;

                        // Prefetch the data slot we're about to write to (reduces write latency)
                        #[cfg(target_arch = "x86_64")]
                        {
                            use std::arch::x86_64::{_mm_prefetch, _MM_HINT_T0};
                            _mm_prefetch(data_ptr as *const i8, _MM_HINT_T0);
                        }

                        // Verify bounds
                        let data_region_size = self.capacity * mem::size_of::<T>();
                        if byte_offset + mem::size_of::<T>() > data_region_size {
                            eprintln!(
                                "Critical safety violation: loan would exceed data region bounds"
                            );
                            return Err("Loan would exceed data region bounds".into());
                        }

                        return Ok(PublisherSample {
                            data_ptr,
                            slot_index: head,
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
    pub fn receive(&self) -> Option<ConsumerSample<'_, T>> {
        let header = unsafe { self.header.as_ref() };

        // MPMC OPTIMIZED: Get this consumer's current tail position from LOCAL MEMORY
        let my_tail = self.consumer_tail.load(Ordering::Relaxed);
        let current_head = header.head.load(Ordering::Acquire);

        // Validate positions
        if my_tail >= self.capacity {
            eprintln!(
                "Critical safety violation: consumer tail {} >= capacity {}",
                my_tail, self.capacity
            );
            return None;
        }

        if current_head >= self.capacity {
            eprintln!(
                "Critical safety violation: head {} >= capacity {}",
                current_head, self.capacity
            );
            return None;
        }

        if my_tail == current_head {
            // No new messages for this consumer
            return None;
        }

        // Calculate next position for this consumer and update in local memory
        let next_tail = (my_tail + 1) % self.capacity;
        self.consumer_tail.store(next_tail, Ordering::Relaxed);

        // Return sample pointing to the message in shared memory
        unsafe {
            let byte_offset = my_tail * mem::size_of::<T>();
            let data_ptr = self.data_ptr.as_ptr().add(byte_offset) as *const T;

            // Prefetch the data we're about to read (reduces read latency)
            #[cfg(target_arch = "x86_64")]
            {
                use std::arch::x86_64::{_mm_prefetch, _MM_HINT_T0};
                _mm_prefetch(data_ptr as *const i8, _MM_HINT_T0);
            }

            // Verify bounds
            let data_region_size = self.capacity * mem::size_of::<T>();
            if byte_offset + mem::size_of::<T>() > data_region_size {
                eprintln!("Critical safety violation: receive would exceed data region bounds");
                return None;
            }

            Some(ConsumerSample {
                data_ptr,
                slot_index: my_tail,
                topic: self,
                _phantom: PhantomData,
            })
        }
    }

    /// Read the most recent message without advancing consumer position
    /// Unlike receive(), this always returns the latest message if one exists,
    /// regardless of when this consumer started reading.
    /// This is useful for reading static/infrequently-updated data.
    pub fn read_latest(&self) -> Option<ConsumerSample<'_, T>> {
        let header = unsafe { self.header.as_ref() };
        let current_head = header.head.load(Ordering::Acquire);

        // If head is 0, no messages have been written yet
        if current_head == 0 {
            return None;
        }

        // The most recent message is at (head - 1) in a ring buffer
        // Handle wrap-around for ring buffer
        let latest_slot = if current_head == 0 {
            self.capacity - 1
        } else {
            current_head - 1
        };

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
        unsafe {
            let byte_offset = latest_slot * mem::size_of::<T>();
            let data_ptr = self.data_ptr.as_ptr().add(byte_offset) as *const T;

            // Verify bounds
            let data_region_size = self.capacity * mem::size_of::<T>();
            if byte_offset + mem::size_of::<T>() > data_region_size {
                eprintln!("Critical safety violation: read_latest would exceed data region bounds");
                return None;
            }

            Some(ConsumerSample {
                data_ptr,
                slot_index: latest_slot,
                topic: self,
                _phantom: PhantomData,
            })
        }
    }

    /// Loan a slot and immediately write data (convenience method)
    /// This is equivalent to loan() followed by write(), but more convenient
    pub fn loan_and_write(&self, value: T) -> Result<(), T> {
        match self.loan() {
            Ok(mut sample) => {
                sample.write(value);
                // Sample is automatically published when dropped
                Ok(())
            }
            Err(_) => Err(value),
        }
    }
}

impl<T> Drop for ShmTopic<T> {
    fn drop(&mut self) {
        // MPMC FIX: Decrement consumer count when this consumer is dropped
        // This prevents the "Maximum number of consumers exceeded" error
        // when consumers exit without proper cleanup
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
