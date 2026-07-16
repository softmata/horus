//! Backend storage enum for per-path optimized data planes.
//!
//! Every topic is SHM-backed, so the data plane always lives in a shared-memory
//! region. Standard SHM backends read/write the main region via cached pointers
//! in `LocalState` (`ShmData`); `FanoutShm` owns a separate SHM-backed SPSC
//! matrix.

use super::shm_fanout::ShmFanoutRing;

/// Per-path optimized backend storage.
///
/// The control plane (SHM header for discovery) is always in shared memory.
/// The data plane is stored here:
///
/// - `ShmData`: Shared memory data region (~50-167ns)
/// - `FanoutShm`: Separate SHM-backed SPSC fan-out matrix
/// - `Uninitialized`: Before first send/recv
pub(crate) enum BackendStorage<T> {
    /// Not yet initialized — will be created on first send/recv
    Uninitialized,
    /// Cross-process — data lives in the SHM region (accessed via LocalState cached pointers)
    ShmData,
    /// Cross-process MPMC — contention-free fan-out via SHM-backed SPSC matrix
    FanoutShm(Box<ShmFanoutRing>),
    /// Never constructed — keeps `BackendStorage` generic over `T` now that all
    /// live variants are type-erased. `PhantomData` carries the `T` binding.
    #[allow(dead_code)]
    _Phantom(std::marker::PhantomData<T>),
}
