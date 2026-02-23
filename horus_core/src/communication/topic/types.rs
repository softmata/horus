//! Type definitions for the topic system.
//!
//! Contains enums and structs shared across all backend implementations.

use std::marker::PhantomData;

// ============================================================================
// Backend Mode Enum
// ============================================================================

/// Selected backend mode stored in shared memory
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BackendMode {
    /// Unknown/uninitialized - will be determined on first use
    Unknown = 0,
    /// DirectChannel - same thread (~3ns)
    DirectChannel = 1,
    /// SpscIntra - same process, 1P1C (~18ns)
    SpscIntra = 2,
    /// SpmcIntra - same process, 1P-MC (~24ns)
    SpmcIntra = 3,
    /// MpscIntra - same process, MP-1C (~26ns)
    MpscIntra = 4,
    /// MpmcIntra - same process, MPMC (~36ns)
    MpmcIntra = 5,
    /// PodShm - cross process, POD type (~50ns)
    PodShm = 6,
    /// MpscShm - cross process, MP-1C (~65ns)
    MpscShm = 7,
    /// SpmcShm - cross process, 1P-MC (~70ns)
    SpmcShm = 8,
    /// SpscShm - cross process, 1P1C (~85ns)
    SpscShm = 9,
    /// MpmcShm - cross process, MPMC (~167ns)
    MpmcShm = 10,
}

impl From<u8> for BackendMode {
    fn from(v: u8) -> Self {
        match v {
            1 => BackendMode::DirectChannel,
            2 => BackendMode::SpscIntra,
            3 => BackendMode::SpmcIntra,
            4 => BackendMode::MpscIntra,
            5 => BackendMode::MpmcIntra,
            6 => BackendMode::PodShm,
            7 => BackendMode::MpscShm,
            8 => BackendMode::SpmcShm,
            9 => BackendMode::SpscShm,
            10 => BackendMode::MpmcShm,
            _ => BackendMode::Unknown,
        }
    }
}

impl BackendMode {
    /// Get the expected latency for this backend mode in nanoseconds
    pub fn expected_latency_ns(&self) -> u64 {
        match self {
            BackendMode::Unknown => 167, // Fallback to MPMC
            BackendMode::DirectChannel => 3,
            BackendMode::SpscIntra => 18,
            BackendMode::SpmcIntra => 24,
            BackendMode::MpscIntra => 26,
            BackendMode::MpmcIntra => 36,
            BackendMode::PodShm => 50,
            BackendMode::MpscShm => 65,
            BackendMode::SpmcShm => 70,
            BackendMode::SpscShm => 85,
            BackendMode::MpmcShm => 167,
        }
    }

    /// Check if this is a cross-process backend
    pub fn is_cross_process(&self) -> bool {
        matches!(
            self,
            BackendMode::PodShm
                | BackendMode::MpscShm
                | BackendMode::SpmcShm
                | BackendMode::SpscShm
                | BackendMode::MpmcShm
        )
    }

    /// Check if this is an intra-process backend
    pub fn is_intra_process(&self) -> bool {
        matches!(
            self,
            BackendMode::DirectChannel
                | BackendMode::SpscIntra
                | BackendMode::SpmcIntra
                | BackendMode::MpscIntra
                | BackendMode::MpmcIntra
        )
    }
}

// ============================================================================
// Topic Role
// ============================================================================

/// Role of a topic participant
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TopicRole {
    /// Not yet registered (first send/recv will determine)
    Unregistered,
    /// Producer only (can send)
    Producer,
    /// Consumer only (can recv)
    Consumer,
    /// Both producer and consumer
    Both,
}

impl TopicRole {
    /// Check if this role can send
    /// HOT PATH: Called on every send() - must be maximally inlined
    #[inline(always)]
    pub fn can_send(&self) -> bool {
        matches!(self, TopicRole::Producer | TopicRole::Both)
    }

    /// Check if this role can receive
    /// HOT PATH: Called on every recv() - must be maximally inlined
    #[inline(always)]
    pub fn can_recv(&self) -> bool {
        matches!(self, TopicRole::Consumer | TopicRole::Both)
    }
}

// ============================================================================
// Connection State
// ============================================================================

/// Connection state for a topic (primarily for network backends)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum ConnectionState {
    /// Not connected
    #[default]
    Disconnected,
    /// Connected and operational
    Connected,
}

impl ConnectionState {
    /// Convert to u8 for atomic storage
    pub fn into_u8(self) -> u8 {
        match self {
            ConnectionState::Disconnected => 0,
            ConnectionState::Connected => 2,
        }
    }

    /// Convert from u8
    #[allow(dead_code)]
    pub fn from_u8(value: u8) -> Self {
        match value {
            2 => ConnectionState::Connected,
            _ => ConnectionState::Disconnected,
        }
    }
}

// ============================================================================
// Topic Descriptor
// ============================================================================

/// Type-safe topic descriptor for compile-time checked topic names.
///
/// Created by the [`topics!`](crate::topics!) macro to provide compile-time
/// type checking for topic names and message types.
#[derive(Debug, Clone, Copy)]
pub struct TopicDescriptor<T> {
    name: &'static str,
    _marker: PhantomData<T>,
}

impl<T> TopicDescriptor<T> {
    /// Create a new topic descriptor (used by the `topics!` macro).
    #[inline]
    pub const fn new(name: &'static str) -> Self {
        Self {
            name,
            _marker: PhantomData,
        }
    }

    /// Get the topic name.
    #[inline]
    pub const fn name(&self) -> &'static str {
        self.name
    }
}
