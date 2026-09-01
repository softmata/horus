//! Type definitions for the topic system.
//!
//! Contains enums and structs shared across all backend implementations.

use std::marker::PhantomData;

// ============================================================================
// Backend Mode Enum
// ============================================================================

/// Selected backend mode stored in shared memory.
///
/// Every real topic is SHM-backed (a backing file is created and `creator_pid`
/// is stamped at header init), so only the cross-process shared-memory backends
/// are ever selected. The numeric discriminants are preserved (not renumbered)
/// so on-disk/SHM headers written by older builds still decode correctly; the
/// retired intra-process discriminants (1–5) now decode to `Unknown`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BackendMode {
    /// Unknown/uninitialized - will be determined on first use
    Unknown = 0,
    /// PodShm - cross process, POD type (~50ns)
    PodShm = 6,
    /// MpscShm - cross process, MP-1C (~65ns)
    MpscShm = 7,
    /// SpmcShm - cross process, 1P-MC (~70ns)
    SpmcShm = 8,
    /// SpscShm - cross process, 1P1C (~85ns)
    SpscShm = 9,
    /// FanoutShm - cross process, contention-free MPMC via SHM SPSC matrix (~40ns)
    FanoutShm = 11,
}

impl BackendMode {
    /// Whether a producer on this backend can observe a full ring and refuse.
    ///
    /// `PodShm` is the odd one out: `send_shm_pod_broadcast` is a bare
    /// `fetch_add` plus an unconditional seqlock overwrite with a single exit,
    /// `Ok(())`. It never reads `header.tail`, so it cannot fail and cannot be
    /// waited on. Every other live backend has an error return for a full ring
    /// (`send_fanout_shm` and `send_shm_mp_pod` two each, `send_shm_sp_pod` one).
    ///
    /// This matters to `send_blocking`, which promises delivery: on a backend
    /// that cannot report fullness the promise is unkeepable, and silently
    /// returning `Ok` on a topic documented for emergency stop is the wrong way
    /// to discover that.
    pub(crate) fn provides_backpressure(self) -> bool {
        match self {
            BackendMode::MpscShm
            | BackendMode::SpmcShm
            | BackendMode::SpscShm
            | BackendMode::FanoutShm => true,
            BackendMode::PodShm => false,
            // Not "no backpressure" -- "not resolved yet". A freshly constructed
            // topic reports Unknown until its first real send settles the header,
            // and treating that as a refusal rejects topics that have simply not
            // decided what they are. Answer permissively and let the next call,
            // by which point the mode is real, make the decision.
            BackendMode::Unknown => true,
        }
    }
}

impl From<u8> for BackendMode {
    fn from(v: u8) -> Self {
        match v {
            // 1..=5 were the retired intra-process backends — decode to Unknown.
            6 => BackendMode::PodShm,
            7 => BackendMode::MpscShm,
            8 => BackendMode::SpmcShm,
            9 => BackendMode::SpscShm,
            10 => BackendMode::FanoutShm, // Legacy MpmcShm value maps to FanoutShm
            11 => BackendMode::FanoutShm,
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
                | BackendMode::FanoutShm
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
