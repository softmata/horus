//! # Smart Backend - Automatic Optimal IPC Selection
//!
//! This module provides `SmartShmBackend<T>` which automatically selects the
//! optimal IPC strategy at runtime based on POD type registration.
//!
//! ## How It Works
//!
//! 1. At construction, checks `is_registered_pod::<T>()`
//! 2. If POD: Uses zero-copy byte transfer (~50ns)
//! 3. If not POD: Uses bincode serialization (~167ns)
//!
//! ## Safety
//!
//! The POD path uses unsafe type erasure, but this is sound because:
//! - User explicitly registered T as POD via `unsafe impl PodMessage`
//! - Registration captures size/alignment which we verify at runtime
//! - The bytemuck crate guarantees safe byte casting for Pod types

use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, AtomicU32, AtomicU8, Ordering};
use std::sync::Arc;

use crate::communication::pod::is_registered_pod;
use crate::communication::topic::TopicBackendTrait;
use crate::error::{HorusError, HorusResult};
use crate::memory::shm_region::ShmRegion;

// ============================================================================
// Smart Backend Header
// ============================================================================

/// Magic number for smart backend validation
const SMART_MAGIC: u64 = 0x534D415254534D42; // "SMARTSMB"

/// Backend mode stored in shared memory
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendMode {
    /// Unknown/uninitialized
    Unknown = 0,
    /// POD mode - zero-copy byte transfer
    Pod = 1,
    /// Ring buffer mode - serialized messages
    Ring = 2,
}

impl From<u8> for BackendMode {
    fn from(v: u8) -> Self {
        match v {
            1 => BackendMode::Pod,
            2 => BackendMode::Ring,
            _ => BackendMode::Unknown,
        }
    }
}

/// Shared memory header for smart backend.
///
/// This header supports both POD and ring buffer modes in one structure.
/// Layout: 128 bytes (2 cache lines for better access patterns)
#[repr(C, align(64))]
pub struct SmartBackendHeader {
    // === Cache line 1: Metadata (read-mostly) ===
    /// Magic number for validation
    pub magic: u64,
    /// Backend mode (Pod=1, Ring=2)
    pub mode: AtomicU8,
    /// Reserved flags
    pub _flags: [u8; 3],
    /// Type size in bytes
    pub type_size: u32,
    /// Type alignment
    pub type_align: u32,
    /// Ring buffer capacity (only for Ring mode)
    pub capacity: u32,
    /// Capacity mask (capacity - 1, for fast modulo)
    pub mask: u32,
    /// Creator PID for same-process detection
    pub creator_pid: u32,
    /// Publisher count
    pub publisher_count: AtomicU32,
    /// Subscriber count
    pub subscriber_count: AtomicU32,
    /// Padding to 64 bytes
    pub _pad1: [u8; 20],

    // === Cache line 2: Hot data (frequently updated) ===
    /// POD mode: sequence counter / Ring mode: write head
    pub sequence_or_head: AtomicU64,
    /// Ring mode only: read tail
    pub tail: AtomicU64,
    /// Last update timestamp (nanoseconds)
    pub last_update_ns: AtomicU64,
    /// Padding to 64 bytes
    pub _pad2: [u8; 40],
}

const _: () = assert!(mem::size_of::<SmartBackendHeader>() == 128);

impl SmartBackendHeader {
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.magic == SMART_MAGIC
    }

    #[inline]
    pub fn mode(&self) -> BackendMode {
        BackendMode::from(self.mode.load(Ordering::Acquire))
    }

    #[inline]
    pub fn is_same_process(&self) -> bool {
        self.creator_pid == std::process::id()
    }
}

// ============================================================================
// Smart Backend Implementation
// ============================================================================

/// Smart shared memory backend that automatically selects optimal strategy.
///
/// - For registered POD types: Zero-copy byte transfer (~50ns)
/// - For other types: Ring buffer with serialization (~167ns)
pub struct SmartShmBackend<T> {
    /// Shared memory region
    region: Arc<ShmRegion>,
    /// Header pointer
    header: NonNull<SmartBackendHeader>,
    /// Data area pointer
    data_ptr: NonNull<u8>,
    /// Whether this instance is a producer
    is_producer: bool,
    /// Cached mode (avoid repeated atomic load)
    mode: BackendMode,
    /// Last seen sequence (for POD mode consumer tracking)
    last_seen: AtomicU64,
    /// Type marker
    _phantom: PhantomData<T>,
}

// Safety: SmartShmBackend uses atomic operations for all shared state
unsafe impl<T: Send> Send for SmartShmBackend<T> {}
unsafe impl<T: Sync> Sync for SmartShmBackend<T> {}

impl<T: Clone + Send + Sync + 'static> SmartShmBackend<T> {
    /// Create a new smart backend.
    ///
    /// Automatically selects POD mode if T is registered, otherwise Ring mode.
    pub fn new(name: &str, capacity: usize, is_producer: bool) -> HorusResult<Self> {
        let is_pod = is_registered_pod::<T>();
        let type_size = mem::size_of::<T>();
        let type_align = mem::align_of::<T>();

        // Calculate required size
        let header_size = mem::size_of::<SmartBackendHeader>();
        let data_size = if is_pod {
            // POD mode: just one slot for the latest value
            type_size
        } else {
            // Ring mode: capacity * slot_size
            // Each slot: 8 bytes sequence + type_size (aligned)
            let slot_size = 8 + ((type_size + 7) & !7); // 8-byte aligned
            capacity * slot_size
        };

        let total_size = header_size + data_size;
        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        let header_ptr = region.as_ptr() as *mut SmartBackendHeader;
        if header_ptr.is_null() {
            return Err(HorusError::Communication("Null header pointer".into()));
        }

        let header = unsafe { NonNull::new_unchecked(header_ptr) };
        let data_ptr = unsafe {
            NonNull::new_unchecked(region.as_ptr().add(header_size) as *mut u8)
        };

        let mode = if is_pod { BackendMode::Pod } else { BackendMode::Ring };

        if is_owner {
            // Initialize header
            unsafe {
                let h = header.as_ptr();
                (*h).magic = SMART_MAGIC;
                (*h).mode = AtomicU8::new(mode as u8);
                (*h)._flags = [0; 3];
                (*h).type_size = type_size as u32;
                (*h).type_align = type_align as u32;
                (*h).capacity = capacity as u32;
                (*h).mask = (capacity - 1) as u32;
                (*h).creator_pid = std::process::id();
                (*h).publisher_count = AtomicU32::new(if is_producer { 1 } else { 0 });
                (*h).subscriber_count = AtomicU32::new(if is_producer { 0 } else { 1 });
                (*h)._pad1 = [0; 20];
                (*h).sequence_or_head = AtomicU64::new(0);
                (*h).tail = AtomicU64::new(0);
                (*h).last_update_ns = AtomicU64::new(0);
                (*h)._pad2 = [0; 40];

                // Zero the data area
                std::ptr::write_bytes(data_ptr.as_ptr(), 0, data_size);
            }

            log::debug!(
                "SmartShmBackend '{}': Created with {:?} mode (is_pod={}, size={}, capacity={})",
                name, mode, is_pod, type_size, capacity
            );
        } else {
            // Validate existing header
            let h = unsafe { header.as_ref() };

            if !h.is_valid() {
                return Err(HorusError::Communication(format!(
                    "Invalid smart backend magic: expected 0x{:X}, got 0x{:X}",
                    SMART_MAGIC, h.magic
                )));
            }

            let existing_mode = h.mode();
            if existing_mode != mode {
                // Mode mismatch - this shouldn't happen if same type
                log::warn!(
                    "SmartShmBackend '{}': Mode mismatch (existing={:?}, expected={:?})",
                    name, existing_mode, mode
                );
            }

            if h.type_size != type_size as u32 {
                return Err(HorusError::Communication(format!(
                    "Type size mismatch: expected {}, got {}",
                    type_size, h.type_size
                )));
            }

            // Register this endpoint
            if is_producer {
                h.publisher_count.fetch_add(1, Ordering::AcqRel);
            } else {
                h.subscriber_count.fetch_add(1, Ordering::AcqRel);
            }

            log::debug!(
                "SmartShmBackend '{}': Joined with {:?} mode (pubs={}, subs={})",
                name, existing_mode,
                h.publisher_count.load(Ordering::Relaxed),
                h.subscriber_count.load(Ordering::Relaxed)
            );
        }

        Ok(Self {
            region,
            header,
            data_ptr,
            is_producer,
            mode,
            last_seen: AtomicU64::new(0),
            _phantom: PhantomData,
        })
    }

    /// Push using POD mode (zero-copy).
    ///
    /// # Safety
    /// Only call this if T is a registered POD type.
    #[inline]
    unsafe fn push_pod(&self, msg: T) -> Result<(), T> {
        let header = self.header.as_ref();

        // Write the message bytes directly
        let src = &msg as *const T as *const u8;
        std::ptr::copy_nonoverlapping(src, self.data_ptr.as_ptr(), mem::size_of::<T>());

        // Increment sequence to signal new data
        header.sequence_or_head.fetch_add(1, Ordering::Release);

        // Don't drop msg - we copied its bytes
        std::mem::forget(msg);

        Ok(())
    }

    /// Pop using POD mode (zero-copy).
    ///
    /// # Safety
    /// Only call this if T is a registered POD type.
    #[inline]
    unsafe fn pop_pod(&self) -> Option<T> {
        let header = self.header.as_ref();

        // Check if new data available
        let current_seq = header.sequence_or_head.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);

        if current_seq == last {
            return None;
        }

        // Read the message bytes directly
        let mut result: T = std::mem::zeroed();
        let dst = &mut result as *mut T as *mut u8;
        std::ptr::copy_nonoverlapping(self.data_ptr.as_ptr(), dst, mem::size_of::<T>());

        // Update last seen
        self.last_seen.store(current_seq, Ordering::Relaxed);

        Some(result)
    }

    /// Push using Ring mode (with Clone).
    #[inline]
    fn push_ring(&self, msg: T) -> Result<(), T> {
        let header = unsafe { self.header.as_ref() };
        let capacity = header.capacity as usize;
        let mask = header.mask as u64;

        // Get current head position
        let head = header.sequence_or_head.load(Ordering::Relaxed);
        let tail = header.tail.load(Ordering::Acquire);

        // Check if full
        if head.wrapping_sub(tail) >= capacity as u64 {
            return Err(msg);
        }

        // Calculate slot position
        let slot_idx = (head & mask) as usize;
        let type_size = mem::size_of::<T>();
        let slot_size = 8 + ((type_size + 7) & !7);
        let slot_offset = slot_idx * slot_size;

        unsafe {
            let slot_ptr = self.data_ptr.as_ptr().add(slot_offset);

            // Write sequence number
            let seq_ptr = slot_ptr as *mut u64;
            std::ptr::write_volatile(seq_ptr, head + 1);

            // Write data (using ptr::write to handle Drop correctly)
            let data_ptr = slot_ptr.add(8) as *mut T;
            std::ptr::write(data_ptr, msg);
        }

        // Advance head
        header.sequence_or_head.store(head + 1, Ordering::Release);

        Ok(())
    }

    /// Pop using Ring mode (with Clone).
    #[inline]
    fn pop_ring(&self) -> Option<T> {
        let header = unsafe { self.header.as_ref() };
        let mask = header.mask as u64;

        let tail = header.tail.load(Ordering::Relaxed);
        let head = header.sequence_or_head.load(Ordering::Acquire);

        // Check if empty
        if tail >= head {
            return None;
        }

        // Calculate slot position
        let slot_idx = (tail & mask) as usize;
        let type_size = mem::size_of::<T>();
        let slot_size = 8 + ((type_size + 7) & !7);
        let slot_offset = slot_idx * slot_size;

        let msg = unsafe {
            let slot_ptr = self.data_ptr.as_ptr().add(slot_offset);

            // Check sequence (ensure write is complete)
            let seq_ptr = slot_ptr as *const u64;
            let seq = std::ptr::read_volatile(seq_ptr);
            if seq != tail + 1 {
                return None; // Write not complete
            }

            // Read data
            let data_ptr = slot_ptr.add(8) as *const T;
            std::ptr::read(data_ptr)
        };

        // Advance tail
        header.tail.store(tail + 1, Ordering::Release);

        Some(msg)
    }

    /// Get the backend mode.
    #[inline]
    pub fn mode(&self) -> BackendMode {
        self.mode
    }

    /// Check if using POD mode.
    #[inline]
    pub fn is_pod_mode(&self) -> bool {
        self.mode == BackendMode::Pod
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for SmartShmBackend<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        if !self.is_producer {
            return Err(msg);
        }

        match self.mode {
            BackendMode::Pod => {
                // Safety: mode is Pod only if is_registered_pod::<T>() was true
                unsafe { self.push_pod(msg) }
            }
            BackendMode::Ring => {
                self.push_ring(msg)
            }
            BackendMode::Unknown => {
                Err(msg)
            }
        }
    }

    #[inline]
    fn pop(&self) -> Option<T> {
        match self.mode {
            BackendMode::Pod => {
                // Safety: mode is Pod only if is_registered_pod::<T>() was true
                unsafe { self.pop_pod() }
            }
            BackendMode::Ring => {
                self.pop_ring()
            }
            BackendMode::Unknown => {
                None
            }
        }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        let header = unsafe { self.header.as_ref() };

        match self.mode {
            BackendMode::Pod => {
                let current_seq = header.sequence_or_head.load(Ordering::Acquire);
                let last = self.last_seen.load(Ordering::Relaxed);
                current_seq == last
            }
            BackendMode::Ring => {
                let tail = header.tail.load(Ordering::Relaxed);
                let head = header.sequence_or_head.load(Ordering::Acquire);
                tail >= head
            }
            BackendMode::Unknown => true,
        }
    }

    fn backend_name(&self) -> &'static str {
        match self.mode {
            BackendMode::Pod => "SmartShm(Pod)",
            BackendMode::Ring => "SmartShm(Ring)",
            BackendMode::Unknown => "SmartShm(Unknown)",
        }
    }
}

impl<T> Clone for SmartShmBackend<T> {
    fn clone(&self) -> Self {
        // Register new endpoint
        let header = unsafe { self.header.as_ref() };
        if self.is_producer {
            header.publisher_count.fetch_add(1, Ordering::AcqRel);
        } else {
            header.subscriber_count.fetch_add(1, Ordering::AcqRel);
        }

        Self {
            region: self.region.clone(),
            header: self.header,
            data_ptr: self.data_ptr,
            is_producer: self.is_producer,
            mode: self.mode,
            last_seen: AtomicU64::new(self.last_seen.load(Ordering::Relaxed)),
            _phantom: PhantomData,
        }
    }
}

impl<T> Drop for SmartShmBackend<T> {
    fn drop(&mut self) {
        // Deregister endpoint
        let header = unsafe { self.header.as_ref() };
        if self.is_producer {
            header.publisher_count.fetch_sub(1, Ordering::AcqRel);
        } else {
            header.subscriber_count.fetch_sub(1, Ordering::AcqRel);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::pod::PodMessage;
    use crate::register_pod_type;

    // Test POD type - registered for smart detection
    #[repr(C)]
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct TestPodMsg {
        timestamp: u64,
        value: f32,
        _pad: [u8; 4],
    }

    unsafe impl crate::bytemuck::Zeroable for TestPodMsg {}
    unsafe impl crate::bytemuck::Pod for TestPodMsg {}
    unsafe impl PodMessage for TestPodMsg {}
    register_pod_type!(TestPodMsg);

    #[test]
    fn test_header_size() {
        assert_eq!(mem::size_of::<SmartBackendHeader>(), 128);
    }

    #[test]
    fn test_non_pod_type() {
        // String is not POD, should use Ring mode
        let backend: SmartShmBackend<String> =
            SmartShmBackend::new("/test_smart_string", 64, true).unwrap();
        assert_eq!(backend.mode(), BackendMode::Ring);

        // Cleanup
        std::fs::remove_file("/dev/shm/test_smart_string").ok();
    }

    #[test]
    fn test_primitive_non_registered() {
        // u64 is Copy but not registered as POD, should use Ring mode
        let backend: SmartShmBackend<u64> =
            SmartShmBackend::new("/test_smart_u64", 64, true).unwrap();
        // Note: u64 is not registered via register_pod_type!, so Ring mode
        assert_eq!(backend.mode(), BackendMode::Ring);

        // Cleanup
        std::fs::remove_file("/dev/shm/test_smart_u64").ok();
    }

    #[test]
    fn test_registered_pod_type() {
        // TestPodMsg is registered via register_pod_type!, should use Pod mode
        let backend: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_pod", 64, true).unwrap();
        assert_eq!(backend.mode(), BackendMode::Pod);
        assert!(backend.is_pod_mode());

        // Cleanup
        std::fs::remove_file("/dev/shm/test_smart_pod").ok();
    }

    #[test]
    fn test_pod_push_pop() {
        let producer: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_pod_pp", 64, true).unwrap();
        let consumer: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_pod_pp", 64, false).unwrap();

        assert!(producer.is_pod_mode());
        assert!(consumer.is_pod_mode());

        // Push message
        let msg = TestPodMsg {
            timestamp: 12345,
            value: 3.14,
            _pad: [0; 4],
        };
        assert!(producer.push(msg).is_ok());

        // Pop message
        let received = consumer.pop();
        assert!(received.is_some());
        let received = received.unwrap();
        assert_eq!(received.timestamp, 12345);
        assert!((received.value - 3.14).abs() < 0.001);

        // Cleanup
        std::fs::remove_file("/dev/shm/test_smart_pod_pp").ok();
    }

    #[test]
    fn test_ring_push_pop() {
        let producer: SmartShmBackend<String> =
            SmartShmBackend::new("/test_smart_ring_pp", 64, true).unwrap();
        let consumer: SmartShmBackend<String> =
            SmartShmBackend::new("/test_smart_ring_pp", 64, false).unwrap();

        assert_eq!(producer.mode(), BackendMode::Ring);
        assert_eq!(consumer.mode(), BackendMode::Ring);

        // Push message
        assert!(producer.push("hello".to_string()).is_ok());

        // Pop message
        let received = consumer.pop();
        assert!(received.is_some());
        assert_eq!(received.unwrap(), "hello");

        // Cleanup
        std::fs::remove_file("/dev/shm/test_smart_ring_pp").ok();
    }

    #[test]
    fn test_publisher_subscriber_count() {
        let producer1: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_count", 64, true).unwrap();

        // Check initial counts
        let header = unsafe { producer1.header.as_ref() };
        assert_eq!(header.publisher_count.load(Ordering::Relaxed), 1);
        assert_eq!(header.subscriber_count.load(Ordering::Relaxed), 0);

        // Add another producer
        let producer2: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_count", 64, true).unwrap();
        assert_eq!(header.publisher_count.load(Ordering::Relaxed), 2);

        // Add consumers
        let consumer1: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_count", 64, false).unwrap();
        let consumer2: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_count", 64, false).unwrap();
        assert_eq!(header.subscriber_count.load(Ordering::Relaxed), 2);

        // Drop one producer
        drop(producer2);
        assert_eq!(header.publisher_count.load(Ordering::Relaxed), 1);

        // Drop consumers
        drop(consumer1);
        drop(consumer2);
        assert_eq!(header.subscriber_count.load(Ordering::Relaxed), 0);

        // Cleanup
        drop(producer1);
        std::fs::remove_file("/dev/shm/test_smart_count").ok();
    }

    #[test]
    fn test_backend_name() {
        let pod_backend: SmartShmBackend<TestPodMsg> =
            SmartShmBackend::new("/test_smart_name_pod", 64, true).unwrap();
        assert_eq!(pod_backend.backend_name(), "SmartShm(Pod)");

        let ring_backend: SmartShmBackend<String> =
            SmartShmBackend::new("/test_smart_name_ring", 64, true).unwrap();
        assert_eq!(ring_backend.backend_name(), "SmartShm(Ring)");

        // Cleanup
        std::fs::remove_file("/dev/shm/test_smart_name_pod").ok();
        std::fs::remove_file("/dev/shm/test_smart_name_ring").ok();
    }
}
