//! Type definitions for the adaptive topic system.
//!
//! Contains enums and structs shared across all backend implementations.

use std::marker::PhantomData;

// ============================================================================
// Backend Mode Enum
// ============================================================================

/// Selected backend mode stored in shared memory
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdaptiveBackendMode {
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

impl From<u8> for AdaptiveBackendMode {
    fn from(v: u8) -> Self {
        match v {
            1 => AdaptiveBackendMode::DirectChannel,
            2 => AdaptiveBackendMode::SpscIntra,
            3 => AdaptiveBackendMode::SpmcIntra,
            4 => AdaptiveBackendMode::MpscIntra,
            5 => AdaptiveBackendMode::MpmcIntra,
            6 => AdaptiveBackendMode::PodShm,
            7 => AdaptiveBackendMode::MpscShm,
            8 => AdaptiveBackendMode::SpmcShm,
            9 => AdaptiveBackendMode::SpscShm,
            10 => AdaptiveBackendMode::MpmcShm,
            _ => AdaptiveBackendMode::Unknown,
        }
    }
}

impl AdaptiveBackendMode {
    /// Get the expected latency for this backend mode in nanoseconds
    pub fn expected_latency_ns(&self) -> u64 {
        match self {
            AdaptiveBackendMode::Unknown => 167, // Fallback to MPMC
            AdaptiveBackendMode::DirectChannel => 3,
            AdaptiveBackendMode::SpscIntra => 18,
            AdaptiveBackendMode::SpmcIntra => 24,
            AdaptiveBackendMode::MpscIntra => 26,
            AdaptiveBackendMode::MpmcIntra => 36,
            AdaptiveBackendMode::PodShm => 50,
            AdaptiveBackendMode::MpscShm => 65,
            AdaptiveBackendMode::SpmcShm => 70,
            AdaptiveBackendMode::SpscShm => 85,
            AdaptiveBackendMode::MpmcShm => 167,
        }
    }

    /// Check if this is a cross-process backend
    pub fn is_cross_process(&self) -> bool {
        matches!(
            self,
            AdaptiveBackendMode::PodShm
                | AdaptiveBackendMode::MpscShm
                | AdaptiveBackendMode::SpmcShm
                | AdaptiveBackendMode::SpscShm
                | AdaptiveBackendMode::MpmcShm
        )
    }

    /// Check if this is an intra-process backend
    pub fn is_intra_process(&self) -> bool {
        matches!(
            self,
            AdaptiveBackendMode::DirectChannel
                | AdaptiveBackendMode::SpscIntra
                | AdaptiveBackendMode::SpmcIntra
                | AdaptiveBackendMode::MpscIntra
                | AdaptiveBackendMode::MpmcIntra
        )
    }
}

// ============================================================================
// Topic Role
// ============================================================================

/// Role of a topic participant
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicRole {
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
pub enum ConnectionState {
    /// Not connected
    #[default]
    Disconnected,
    /// Connection in progress
    Connecting,
    /// Connected and operational
    Connected,
    /// Reconnecting after a disconnect
    Reconnecting,
    /// Connection permanently failed
    Failed,
}

impl ConnectionState {
    /// Convert to u8 for atomic storage
    pub fn into_u8(self) -> u8 {
        match self {
            ConnectionState::Disconnected => 0,
            ConnectionState::Connecting => 1,
            ConnectionState::Connected => 2,
            ConnectionState::Reconnecting => 3,
            ConnectionState::Failed => 4,
        }
    }

    /// Convert from u8
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => ConnectionState::Disconnected,
            1 => ConnectionState::Connecting,
            2 => ConnectionState::Connected,
            3 => ConnectionState::Reconnecting,
            4 => ConnectionState::Failed,
            _ => ConnectionState::Disconnected,
        }
    }
}

// ============================================================================
// Backend Hint
// ============================================================================

/// Backend hint for topic creation (kept for API compatibility, ignored by AdaptiveTopic)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BackendHint {
    #[default]
    Auto,
    DirectChannel,
    SpscIntra,
    SpscShm,
    PodShm,
    MpmcShm,
    MpscIntra,
    SpmcIntra,
    MpmcIntra,
    MpscShm,
    SpmcShm,
    Network,
    NetworkZenoh,
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

// ============================================================================
// Topic Config
// ============================================================================

/// Topic configuration for creating topics with specific settings.
///
/// Used primarily by Python bindings and config-file-based topic creation.
pub struct TopicConfig {
    /// Topic name
    pub name: String,
    /// Ring buffer capacity
    pub capacity: u32,
    /// Whether to create (vs open existing)
    pub create: bool,
    /// Whether this is the producer side
    pub is_producer: bool,
}

impl TopicConfig {
    /// Create a new topic configuration
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            capacity: 64,
            create: true,
            is_producer: true,
        }
    }

    /// Set the ring buffer capacity
    pub fn with_capacity(mut self, capacity: u32) -> Self {
        self.capacity = capacity;
        self
    }

    /// Set the backend hint (ignored — AdaptiveTopic auto-selects)
    pub fn with_backend(self, _hint: BackendHint) -> Self {
        // Backend hint is ignored - AdaptiveTopic auto-selects optimal backend
        self
    }
}
