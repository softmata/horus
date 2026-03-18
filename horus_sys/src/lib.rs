//! # horus_sys — Platform Abstraction Layer
//!
//! Provides unified APIs for OS-specific operations used by the HORUS robotics
//! framework. All `#[cfg(target_os)]` code lives here — horus_core and
//! horus_manager import from horus_sys and never use platform APIs directly.
//!
//! ## Modules
//!
//! | Module | Purpose |
//! |--------|---------|
//! | [`shm`] | Shared memory IPC (zero-copy cross-process) |
//! | [`rt`] | Real-time scheduling (priority, memory lock, affinity) |
//! | [`process`] | Process signals, lifecycle, session/user IDs |
//! | [`fs`] | Symlinks, file locks, permissions, dev_null |
//! | [`time`] | High-resolution monotonic timing and precise sleep |
//! | [`discover`] | Node and process discovery |
//! | [`terminal`] | Raw mode detection, color support |
//! | [`device`] | Serial port and video device enumeration |
//! | [`platform`] | OS/distro detection, directories, shell, package manager |
//! | [`sync`] | Environment synchronization (toolchains, system deps) |
//!
//! ## Platform Support
//!
//! Each module has per-platform backends selected at compile time:
//! - **Linux**: Full support (SCHED_FIFO, /dev/shm, seccomp, /proc)
//! - **macOS**: Near-full (Mach threads, shm_open, IOKit)
//! - **Windows**: Near-full (REALTIME_PRIORITY, CreateFileMapping, WMI)

pub mod device;
pub mod discover;
pub mod fs;
pub mod platform;
pub mod process;
pub mod rt;
pub mod shm;
pub mod sync;
pub mod terminal;
pub mod time;
