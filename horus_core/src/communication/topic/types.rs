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

    // --- GPU CUDA IPC variants (feature-gated at usage sites) ---
    /// CudaIpcSpsc - cross process CUDA IPC, 1P1C
    ///
    /// Tensor descriptor goes through SHM ring; actual GPU data is accessed
    /// by the receiver via CUDA IPC handle (zero-copy across processes).
    CudaIpcSpsc = 11,
    /// CudaIpcSpmc - cross process CUDA IPC, 1P-MC
    CudaIpcSpmc = 12,
    /// CudaIpcMpsc - cross process CUDA IPC, MP-1C
    CudaIpcMpsc = 13,
    /// CudaIpcMpmc - cross process CUDA IPC, MPMC
    CudaIpcMpmc = 14,
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
            11 => BackendMode::CudaIpcSpsc,
            12 => BackendMode::CudaIpcSpmc,
            13 => BackendMode::CudaIpcMpsc,
            14 => BackendMode::CudaIpcMpmc,
            _ => BackendMode::Unknown,
        }
    }
}

impl BackendMode {
    /// Check if this is a cross-process backend
    pub fn is_cross_process(&self) -> bool {
        matches!(
            self,
            BackendMode::PodShm
                | BackendMode::MpscShm
                | BackendMode::SpmcShm
                | BackendMode::SpscShm
                | BackendMode::MpmcShm
                | BackendMode::CudaIpcSpsc
                | BackendMode::CudaIpcSpmc
                | BackendMode::CudaIpcMpsc
                | BackendMode::CudaIpcMpmc
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
    #[cfg(test)]
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
