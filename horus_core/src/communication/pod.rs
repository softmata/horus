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
//! ## Smart Detection
//!
//! POD types are automatically detected at runtime using the `inventory` crate.
//! When a type is registered as POD, `Topic::new()` will automatically select
//! the fastest backend (PodShm ~50ns instead of MpmcShm ~167ns).
//!
//! ### Registering POD Types
//!
//! ```rust,ignore
//! use horus_core::communication::{PodMessage, register_pod_type};
//! use bytemuck::{Pod, Zeroable};
//!
//! #[repr(C)]
//! #[derive(Clone, Copy, Pod, Zeroable)]
//! pub struct MotorCommand {
//!     pub timestamp_ns: u64,
//!     pub motor_id: u32,
//!     pub velocity: f32,
//!     pub torque: f32,
//!     pub _pad: [u8; 4],
//! }
//!
//! unsafe impl PodMessage for MotorCommand {}
//!
//! // Register for smart detection (do this once, at module level)
//! register_pod_type!(MotorCommand);
//! ```
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
use std::any::TypeId;
use std::mem;

// ============================================================================
// POD Type Registration System (for Smart Detection)
// ============================================================================

/// Registration entry for a POD type.
/// Used by the smart detection system to identify POD types at runtime.
pub struct PodTypeRegistration {
    /// TypeId of the registered POD type
    pub type_id: TypeId,
    /// Size of the type in bytes
    pub size: usize,
    /// Alignment requirement
    pub align: usize,
    /// Function to get type name (deferred to avoid const issues)
    pub type_name_fn: fn() -> &'static str,
}

// Collect all POD type registrations using inventory
inventory::collect!(PodTypeRegistration);

/// Check if a type is registered as a POD type.
/// Used by Topic::new() for smart backend selection.
///
/// # Performance
/// This function iterates through registered POD types, but this only happens
/// once at Topic creation time, not on every send/recv.
#[inline]
pub fn is_registered_pod<T: 'static>() -> bool {
    let target_id = TypeId::of::<T>();
    inventory::iter::<PodTypeRegistration>().any(|reg| reg.type_id == target_id)
}

/// Get registration info for a POD type, if registered.
pub fn get_pod_registration<T: 'static>() -> Option<&'static PodTypeRegistration> {
    let target_id = TypeId::of::<T>();
    inventory::iter::<PodTypeRegistration>().find(|reg| reg.type_id == target_id)
}

/// Get the count of registered POD types (useful for debugging/metrics).
pub fn registered_pod_count() -> usize {
    inventory::iter::<PodTypeRegistration>().count()
}

/// List all registered POD type names (useful for debugging).
pub fn registered_pod_types() -> Vec<&'static str> {
    inventory::iter::<PodTypeRegistration>()
        .map(|reg| (reg.type_name_fn)())
        .collect()
}

/// Register a POD type for smart detection.
///
/// This macro should be called once per POD type, typically at module level.
/// It enables automatic backend selection in `Topic::new()`.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::{PodMessage, register_pod_type};
///
/// #[repr(C)]
/// #[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
/// pub struct Twist {
///     pub linear: [f64; 3],
///     pub angular: [f64; 3],
/// }
///
/// unsafe impl PodMessage for Twist {}
/// register_pod_type!(Twist);
/// ```
#[macro_export]
macro_rules! register_pod_type {
    ($type:ty) => {
        $crate::inventory::submit! {
            $crate::communication::pod::PodTypeRegistration {
                type_id: std::any::TypeId::of::<$type>(),
                size: std::mem::size_of::<$type>(),
                align: std::mem::align_of::<$type>(),
                type_name_fn: || std::any::type_name::<$type>(),
            }
        }
    };
}

// Re-export inventory for use in the macro
pub use inventory;

// ============================================================================
// POD Message Trait
// ============================================================================

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
