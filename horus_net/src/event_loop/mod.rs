//! Event-driven I/O — epoll (Linux) + kqueue (macOS).
//!
//! No busy-polling. Zero wakeups when idle.

#[cfg(target_os = "linux")]
pub mod epoll;

#[cfg(target_os = "macos")]
pub mod kqueue;

/// Event source — what woke us up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventSource {
    /// UDP socket has data ready to read (import path).
    UdpSocket,
    /// SHM eventfd/pipe signaled — new local data to export.
    ShmNotify,
    /// Timer fired — periodic tasks (discovery, heartbeat).
    Timer,
}

/// A single event from the event loop.
#[derive(Debug, Clone, Copy)]
pub struct Event {
    pub source: EventSource,
}

/// Platform-dispatched event loop.
/// On Linux: epoll. On macOS: kqueue. On other: polling fallback.
#[cfg(target_os = "linux")]
pub type PlatformEventLoop = epoll::EpollLoop;

#[cfg(target_os = "macos")]
pub type PlatformEventLoop = kqueue::KqueueLoop;
