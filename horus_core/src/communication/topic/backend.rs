//! Backend storage enum for per-path optimized data planes.
//!
//! Each variant holds its own maximally-optimized data structure:
//! - Intra-process backends use heap memory (no mmap overhead)
//! - Cross-process backends use a separate SHM data region

use std::sync::Arc;

use super::direct_channel::DirectSlot;
use super::mpmc_intra::MpmcRing;
use super::mpsc_intra::MpscRing;
use super::shm_data::ShmDataBackend;
use super::spmc_intra::SpmcRing;
use super::spsc_intra::SpscRing;

/// Per-path optimized backend storage.
///
/// The control plane (SHM header for discovery) is always in shared memory.
/// The data plane is stored here, with the optimal structure per backend:
///
/// - `DirectChannel`: Heap, header atomics for signaling (~3ns)
/// - `SpscIntra`: Heap, 2 cache-padded atomics (~18ns)
/// - `SpmcIntra`: Heap, seqlock broadcast (~24ns)
/// - `MpscIntra`: Heap, CAS head (~26ns)
/// - `MpmcIntra`: Heap, CAS head+tail (~36ns)
/// - `ShmData`: Shared memory data region (~50-167ns)
/// - `Uninitialized`: Before first send/recv
pub(crate) enum BackendStorage<T> {
    /// Not yet initialized — will be created on first send/recv
    Uninitialized,
    /// Same-thread 1P1C — heap ring, header atomics for ordering
    DirectChannel(Arc<DirectSlot<T>>),
    /// Same-process 1P1C — heap, cache-line separated head/tail
    SpscIntra(Arc<SpscRing<T>>),
    /// Same-process 1P-MC — heap, seqlock broadcast
    SpmcIntra(Arc<SpmcRing<T>>),
    /// Same-process MP-1C — heap, CAS head
    MpscIntra(Arc<MpscRing<T>>),
    /// Same-process MPMC — heap, CAS head+tail
    MpmcIntra(Arc<MpmcRing<T>>),
    /// Cross-process — shared memory data region
    ShmData(ShmDataBackend),
}
