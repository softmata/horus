//! # POD (Plain Old Data) Message System
//!
//! Ultra-fast zero-serialization messaging for real-time robotics control loops.
//!
//! ## Automatic POD Detection
//!
//! HORUS automatically detects POD types using `std::mem::needs_drop::<T>()`.
//! No registration or special traits required.
//!
//! ### ⚠️ This detection is a heuristic, and it is not sound for every type
//!
//! `!needs_drop` proves a type owns no heap allocation. It does NOT prove every
//! bit pattern is a valid value of that type. A struct containing a `bool`, a
//! fieldless enum, or a `NonZeroU32` passes the heuristic and is then
//! transported by raw byte reinterpretation out of shared memory that a peer
//! process — possibly a different build, a stale region, or horus_net's
//! unauthenticated network feed — wrote. Materialising a `bool` holding `0x02`
//! or an out-of-range discriminant is undefined behaviour, and nothing on this
//! path validates it.
//!
//! Until the zero-copy path is gated on an explicit `PodMessage` opt-in (whose
//! `bytemuck::Pod` bound rejects exactly these field types at compile time),
//! **prefer integer fields with explicit encodings over `bool`/enum fields in
//! messages you intend to send as fixed-size** — `u8` flags, as
//! `horus_types::diagnostics` already does. This paragraph used to advertise
//! the heuristic without the caveat.
//!
//! ```rust,ignore
//! // Just define your struct - HORUS auto-detects it as POD
//! struct MotorCommand {
//!     velocity: f32,
//!     torque: f32,
//! }
//!
//! // Use it directly - zero-copy path selected automatically
//! let topic: Topic<MotorCommand> = Topic::new("motor")?;
//! topic.send(MotorCommand { velocity: 1.0, torque: 0.5 });
//! ```
//!
//! ## How Auto-Detection Works
//!
//! A type is POD if:
//! - `!std::mem::needs_drop::<T>()` - No destructor (no heap pointers)
//! - `std::mem::size_of::<T>() > 1` - Not a ZST, and not a single byte
//!
//! This automatically excludes String, Vec, Box, and any type containing them.
//! It does NOT exclude a larger type that *contains* a field with a validity
//! invariant — see the warning above.
//!
//! ## Performance Characteristics
//!
//! | Type | Detection | Latency |
//! |------|-----------|---------|
//! | POD (auto-detected) | Automatic | ~50ns |
//! | Non-POD | Automatic | ~167ns (bincode) |
//!
//! ## PodMessage Trait (Optional)
//!
//! For explicit POD types with bytemuck guarantees, you can still use `PodMessage`:
//!
//! ```rust,ignore
//! use horus_core::communication::PodMessage;
//! use bytemuck::{Pod, Zeroable};
//!
//! #[repr(C)]
//! #[derive(Clone, Copy, Pod, Zeroable)]
//! pub struct MotorCommand {
//!     pub timestamp_ns: u64,
//!     pub velocity: f32,
//!     pub torque: f32,
//!     pub _pad: [u8; 4],
//! }
//!
//! unsafe impl PodMessage for MotorCommand {}
//! ```
//!
//! This provides additional methods like `as_bytes()`, `from_bytes()`, etc.

use crate::types::Tensor;
use bytemuck::{Pod, Zeroable};
use std::mem;

use crate::core::LogSummary;

// ============================================================================
// Automatic POD Detection
// ============================================================================

/// Automatically detect if a type is POD-safe for zero-copy IPC.
///
/// A type is considered POD if:
/// - It has no destructor (`!needs_drop::<T>()`)
/// - It's not a zero-sized type (`size_of::<T>() > 0`)
/// - It's not a single byte (excludes `bool` which has validity invariants —
///   only 0/1 are valid, arbitrary SHM bytes like 0x02 would be instant UB)
///
/// # Known unsoundness (not a mere limitation)
///
/// The `size > 1` filter only rejects types that are *themselves* one byte. A
/// struct whose FIELD is a `bool`, a fieldless enum, or a `NonZeroU32` is
/// larger than one byte and has no destructor, so it is classified POD here and
/// routed to the raw byte-reinterpretation path — which builds a `T` out of
/// bytes another process wrote, with no validation and no `bytemuck::Pod` bound
/// to catch it. An invalid `bool` or discriminant is UB the moment it is
/// materialised.
///
/// This cannot be fixed inside this function: dispatch is a runtime `bool` and
/// `RingTopic<T>` is generic over serde bounds, so there is no `PodMessage`
/// bound available to test. The real fix is an explicit opt-in registry keyed
/// on `TypeId`, populated only by types that assert `PodMessage` (and a
/// `message! { #[fixed] }` macro arm that derives `bytemuck::Pod`, making a
/// `bool` field a compile error). Until then, callers must not put fields with
/// validity invariants in fixed-size messages; the module header says so.
///
/// The torn-read window is separately closed on the seqlock consumer path,
/// which keeps a slot copy as `MaybeUninit<T>` until after its stamp re-check
/// (`simd_aware_read_uninit`), so a discarded torn read never becomes a `T`.
///
/// Types with destructors (String, Vec, Box, etc.) contain heap pointers
/// that become invalid in cross-process communication, so they're excluded.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::is_pod;
///
/// struct MyPod { x: f32, y: f32 }
/// struct NotPod { name: String }
///
/// assert!(is_pod::<MyPod>());      // true - no Drop, size > 1
/// assert!(!is_pod::<NotPod>());    // false - String has Drop
/// assert!(is_pod::<[f32; 3]>());   // true - primitives are POD
/// assert!(!is_pod::<Vec<u8>>());   // false - Vec has Drop
/// assert!(!is_pod::<bool>());      // false - excluded (validity invariant)
/// ```
#[inline]
pub(crate) fn is_pod<T: 'static>() -> bool {
    let size = mem::size_of::<T>();
    // Exclude size <= 1: this catches `bool` (only 0/1 valid), single-byte enums,
    // and ZSTs. Types like `u8` and `i8` are safe but rarely used as IPC messages
    // on their own — the minor false-negative is worth preventing UB from `bool`.
    !mem::needs_drop::<T>() && size > 1
}

// ============================================================================
// POD Message Trait
// ============================================================================

/// Marker trait for messages that can be transferred without serialization.
///
/// Most users should use the `message!` macro or standard message types
/// instead of implementing this trait directly.
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
#[doc(hidden)]
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

// Tensor is repr(C), Pod, Zeroable - safe for zero-copy IPC
unsafe impl PodMessage for Tensor {}

// Unified vision/perception types — repr(C), Pod, zero-copy IPC
unsafe impl PodMessage for crate::types::ImageDescriptor {}
unsafe impl PodMessage for crate::types::PointCloudDescriptor {}
unsafe impl PodMessage for crate::types::DepthImageDescriptor {}

// ============================================================================
// LogSummary for unified types (orphan rule: LogSummary defined here in horus_core)
// ============================================================================

impl LogSummary for crate::types::ImageEncoding {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for crate::types::ImageDescriptor {
    fn log_summary(&self) -> String {
        format!(
            "Image({}x{}, {:?}, {}, {})",
            self.width(),
            self.height(),
            self.encoding(),
            self.dtype(),
            if self.is_cuda() { "cuda" } else { "cpu" },
        )
    }
}

impl LogSummary for crate::types::PointCloudDescriptor {
    fn log_summary(&self) -> String {
        let kind = match self.fields_per_point() {
            3 => "XYZ",
            4 => "XYZI",
            6 => "XYZRGB",
            k => return format!("PointCloud({} pts, {} fields)", self.point_count(), k),
        };
        format!(
            "PointCloud({} pts, {}, {:?})",
            self.point_count(),
            kind,
            self.dtype(),
        )
    }
}

impl LogSummary for crate::types::DepthImageDescriptor {
    fn log_summary(&self) -> String {
        let unit = if self.is_meters() {
            "m"
        } else if self.is_millimeters() {
            "mm"
        } else {
            "?"
        };
        format!("DepthImage({}x{}, {})", self.width(), self.height(), unit,)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[repr(C)]
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct TestMsg {
        timestamp_ns: u64,
        value: f32,
        _pad: [u8; 4],
    }

    unsafe impl Zeroable for TestMsg {}
    unsafe impl Pod for TestMsg {}
    unsafe impl PodMessage for TestMsg {}

    #[test]
    fn test_pod_message_bytes() {
        let msg = TestMsg {
            timestamp_ns: 12345,
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
        assert_eq!(msg.timestamp_ns, 0);
        assert_eq!(msg.value, 0.0);
    }
}
