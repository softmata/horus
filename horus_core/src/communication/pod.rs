//! # POD (Plain Old Data) Message System
//!
//! Ultra-fast zero-serialization messaging for real-time robotics control loops.
//!
//! This module provides a `PodMessage` trait that enables zero-copy message transfer
//! by bypassing serialization entirely. Messages implementing this trait are copied
//! directly as raw bytes, achieving ~50ns latency vs ~250ns with bincode.
//!
//! ## Performance Characteristics
//!
//! | Method | Latency | Use Case |
//! |--------|---------|----------|
//! | POD (this module) | ~50ns | Hard real-time control loops |
//! | Bincode (default) | ~250ns | General sensor/state data |
//! | MessagePack | ~4μs | Cross-language (Python) |
//!
//! ## Safety Requirements
//!
//! POD messages must satisfy strict requirements:
//! - `#[repr(C)]` - C-compatible memory layout
//! - `Copy` - Bitwise copyable
//! - `bytemuck::Pod` - Safe to cast from/to bytes
//! - No padding bytes that could leak data
//! - Fixed size known at compile time
//!
//! ## Example
//!
//! ```rust,ignore
//! use horus_core::communication::PodMessage;
//! use bytemuck::{Pod, Zeroable};
//!
//! #[repr(C)]
//! #[derive(Clone, Copy, Pod, Zeroable)]
//! pub struct MotorCommand {
//!     pub timestamp_ns: u64,
//!     pub motor_id: u32,
//!     pub velocity: f32,
//!     pub torque: f32,
//!     pub _pad: [u8; 4],  // Explicit padding to cache line boundary
//! }
//!
//! // Implement the marker trait
//! unsafe impl PodMessage for MotorCommand {}
//! ```
//!
//! ## Trade-offs
//!
//! **Pros:**
//! - 5x faster than bincode serialization
//! - Zero allocation, zero copying (direct memcpy)
//! - Predictable, constant-time transfer
//! - Cache-line aligned for optimal CPU performance
//!
//! **Cons:**
//! - No schema evolution - struct changes break compatibility
//! - Platform-dependent (endianness, padding)
//! - Requires unsafe trait implementation
//! - Fixed-size only (no Vec, String, etc.)

use bytemuck::{Pod, Zeroable};
use std::mem;

/// Marker trait for messages that can be transferred without serialization.
///
/// # Safety
///
/// Implementing this trait asserts that the type:
/// 1. Has `#[repr(C)]` layout
/// 2. Contains no padding bytes (or padding is explicitly zeroed)
/// 3. Is safe to transmute to/from `[u8; size_of::<Self>()]`
/// 4. Has the same layout across all compilation targets you support
///
/// The type must also implement `Pod + Zeroable` from bytemuck.
pub unsafe trait PodMessage: Pod + Zeroable + Copy + Clone + Send + Sync + 'static {
    /// Size of this message in bytes (compile-time constant)
    const SIZE: usize = mem::size_of::<Self>();

    /// Alignment requirement for this message
    const ALIGN: usize = mem::align_of::<Self>();

    /// Convert message to bytes (zero-copy reference)
    #[inline(always)]
    fn as_bytes(&self) -> &[u8] {
        bytemuck::bytes_of(self)
    }

    /// Convert bytes to message (zero-copy reference)
    ///
    /// # Safety
    /// The slice must have exactly `SIZE` bytes and proper alignment.
    #[inline(always)]
    fn from_bytes(bytes: &[u8]) -> Option<&Self> {
        if bytes.len() != Self::SIZE {
            return None;
        }
        bytemuck::try_from_bytes(bytes).ok()
    }

    /// Create a zeroed instance (all bytes zero)
    #[inline(always)]
    fn zeroed() -> Self {
        Zeroable::zeroed()
    }

    /// Copy message to a byte slice (fast memcpy)
    ///
    /// # Safety
    /// The destination must have at least `SIZE` bytes and proper alignment.
    #[inline(always)]
    unsafe fn write_to_ptr(&self, ptr: *mut u8) {
        std::ptr::copy_nonoverlapping(self.as_bytes().as_ptr(), ptr, Self::SIZE);
    }

    /// Read message from a byte slice (fast memcpy)
    ///
    /// # Safety
    /// The source must have at least `SIZE` bytes and proper alignment.
    #[inline(always)]
    unsafe fn read_from_ptr(ptr: *const u8) -> Self {
        let mut result: Self = <Self as PodMessage>::zeroed();
        std::ptr::copy_nonoverlapping(
            ptr,
            bytemuck::bytes_of_mut(&mut result).as_mut_ptr(),
            Self::SIZE,
        );
        result
    }
}

/// POD-specific Link for ultra-fast SPSC communication
///
/// This is a specialized version of Link that only works with POD messages,
/// bypassing all serialization for minimum latency.
///
/// # Deprecation Notice
///
/// `PodLink<T>` is deprecated in favor of `PodTopic<T>`. The PodTopic API provides
/// the same zero-copy performance with a cleaner interface.
///
/// ## Migration Guide
///
/// ```rust,ignore
/// // Before (deprecated):
/// let producer: PodLink<MotorCommand> = PodLink::producer("motor_cmd")?;
/// producer.send(&cmd)?;
///
/// // After (recommended):
/// let producer: PodTopic<MotorCommand> = PodTopic::producer("motor_cmd")?;
/// producer.send(cmd)?;
/// ```
///
/// PodTopic provides equivalent performance (~50ns) with simpler semantics.
#[deprecated(
    since = "0.2.0",
    note = "Use PodTopic<T> instead. See PodLink documentation for migration guide."
)]
pub struct PodLink<T: PodMessage> {
    /// Shared memory region
    shm_ptr: *mut u8,
    /// Sequence counter for detecting new data
    sequence: *mut std::sync::atomic::AtomicU64,
    /// Last seen sequence (consumer only)
    last_seen: std::sync::atomic::AtomicU64,
    /// Role (producer or consumer)
    is_producer: bool,
    /// Topic name for diagnostics
    topic_name: String,
    /// Phantom for type safety
    _phantom: std::marker::PhantomData<T>,
}

// Safety: PodLink uses atomic operations for synchronization
unsafe impl<T: PodMessage> Send for PodLink<T> {}
unsafe impl<T: PodMessage> Sync for PodLink<T> {}

/// Header for POD Link shared memory region
#[repr(C, align(64))]
struct PodLinkHeader {
    /// Sequence counter - incremented on each write
    sequence: std::sync::atomic::AtomicU64,
    /// Size of the data type (for validation)
    element_size: u64,
    /// Magic number for validation
    magic: u64,
    /// Padding to cache line
    _pad: [u8; 40],
}

const POD_LINK_MAGIC: u64 = 0x484F525553504F44; // "HORUSPOD"

impl<T: PodMessage> PodLink<T> {
    /// Create a POD Link producer
    ///
    /// # Example
    /// ```rust,ignore
    /// let link: PodLink<CmdVel> = PodLink::producer("motor_cmd")?;
    /// link.send(CmdVel::new(1.0, 0.5));
    /// ```
    pub fn producer(topic: &str) -> Result<Self, String> {
        Self::create(topic, true)
    }

    /// Create a POD Link consumer
    ///
    /// # Example
    /// ```rust,ignore
    /// let link: PodLink<CmdVel> = PodLink::consumer("motor_cmd")?;
    /// if let Some(cmd) = link.recv() {
    ///     apply_velocity(cmd);
    /// }
    /// ```
    pub fn consumer(topic: &str) -> Result<Self, String> {
        Self::create(topic, false)
    }

    fn create(topic: &str, is_producer: bool) -> Result<Self, String> {
        use crate::memory::shm_region::ShmRegion;

        // Calculate required size: header + data
        let header_size = mem::size_of::<PodLinkHeader>();
        let data_size = T::SIZE;
        let total_size = header_size + data_size;

        // Ensure cache-line alignment
        let total_size = (total_size + 63) & !63;

        // Create/open shared memory (ShmRegion::new handles both create and open)
        let shm_name = format!("pod/{}", topic);
        let shm = ShmRegion::new(&shm_name, total_size)
            .map_err(|e| format!("Failed to create/open shared memory for '{}': {}", topic, e))?;

        let base_ptr = shm.as_ptr() as *mut u8;

        // Initialize header if producer
        let header_ptr = base_ptr as *mut PodLinkHeader;
        if is_producer {
            unsafe {
                (*header_ptr).magic = POD_LINK_MAGIC;
                (*header_ptr).element_size = T::SIZE as u64;
                (*header_ptr)
                    .sequence
                    .store(0, std::sync::atomic::Ordering::Release);
            }
        } else {
            // Validate header
            unsafe {
                if (*header_ptr).magic != POD_LINK_MAGIC {
                    return Err(format!("Invalid POD Link magic for topic '{}'", topic));
                }
                if (*header_ptr).element_size != T::SIZE as u64 {
                    return Err(format!(
                        "Size mismatch for topic '{}': expected {}, got {}",
                        topic,
                        T::SIZE,
                        (*header_ptr).element_size
                    ));
                }
            }
        }

        let sequence_ptr =
            unsafe { &(*header_ptr).sequence as *const _ as *mut std::sync::atomic::AtomicU64 };
        let data_ptr = unsafe { base_ptr.add(header_size) };

        // Leak the ShmRegion to keep it alive (will be cleaned up on process exit)
        std::mem::forget(shm);

        Ok(Self {
            shm_ptr: data_ptr,
            sequence: sequence_ptr,
            last_seen: std::sync::atomic::AtomicU64::new(0),
            is_producer,
            topic_name: topic.to_string(),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Ultra-fast send - direct memcpy, no serialization (~50ns)
    ///
    /// Overwrites the current value. Single-slot design optimized for
    /// real-time control where only the latest value matters.
    #[inline(always)]
    pub fn send(&self, msg: T) {
        debug_assert!(self.is_producer, "Cannot send on consumer");

        // Direct memory write - no serialization!
        unsafe {
            T::write_to_ptr(&msg, self.shm_ptr);
        }

        // Increment sequence with Release ordering to publish
        unsafe {
            (*self.sequence).fetch_add(1, std::sync::atomic::Ordering::Release);
        }
    }

    /// Ultra-fast receive - direct memcpy, no deserialization (~50ns)
    ///
    /// Returns `Some(msg)` if new data available, `None` if already seen.
    /// Single-slot design: always returns the latest value.
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        debug_assert!(!self.is_producer, "Cannot recv on producer");

        // Check sequence with Acquire ordering
        let current_seq = unsafe { (*self.sequence).load(std::sync::atomic::Ordering::Acquire) };

        let last_seen = self.last_seen.load(std::sync::atomic::Ordering::Relaxed);

        if current_seq == last_seen {
            return None; // No new data
        }

        // Read data directly - no deserialization!
        let msg = unsafe { T::read_from_ptr(self.shm_ptr) };

        // Update last seen
        self.last_seen
            .store(current_seq, std::sync::atomic::Ordering::Relaxed);

        Some(msg)
    }

    /// Blocking receive with spin-wait
    ///
    /// Spins until new data is available. Use for tight control loops
    /// where latency is critical and CPU usage is acceptable.
    #[inline(always)]
    pub fn recv_blocking(&self) -> T {
        loop {
            if let Some(msg) = self.recv() {
                return msg;
            }
            std::hint::spin_loop();
        }
    }

    /// Get topic name
    pub fn topic(&self) -> &str {
        &self.topic_name
    }

    /// Check if this is a producer
    pub fn is_producer(&self) -> bool {
        self.is_producer
    }
}

impl<T: PodMessage> std::fmt::Debug for PodLink<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PodLink")
            .field("topic", &self.topic_name)
            .field(
                "role",
                &if self.is_producer {
                    "producer"
                } else {
                    "consumer"
                },
            )
            .field("element_size", &T::SIZE)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[repr(C)]
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct TestMsg {
        timestamp: u64,
        value: f32,
        _pad: [u8; 4],
    }

    unsafe impl Zeroable for TestMsg {}
    unsafe impl Pod for TestMsg {}
    unsafe impl PodMessage for TestMsg {}

    #[test]
    fn test_pod_message_bytes() {
        let msg = TestMsg {
            timestamp: 12345,
            value: 3.125,
            _pad: [0; 4],
        };

        let bytes = msg.as_bytes();
        assert_eq!(bytes.len(), TestMsg::SIZE);

        let restored = TestMsg::from_bytes(bytes).unwrap();
        assert_eq!(*restored, msg);
    }

    #[test]
    fn test_pod_message_size() {
        assert_eq!(TestMsg::SIZE, 16); // 8 + 4 + 4 = 16 bytes
    }

    #[test]
    fn test_pod_message_zeroed() {
        let msg: TestMsg = <TestMsg as PodMessage>::zeroed();
        assert_eq!(msg.timestamp, 0);
        assert_eq!(msg.value, 0.0);
    }
}
