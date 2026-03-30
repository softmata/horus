//! Transport layer — pluggable trait with UDP implementation.

pub mod udp;

use std::io;
use std::net::SocketAddr;

/// Transport trait — abstraction over the network socket.
pub trait Transport {
    /// Send data to a specific address.
    fn send_to(&self, data: &[u8], addr: SocketAddr) -> io::Result<usize>;

    /// Receive data. Returns (bytes_read, source_address).
    fn recv_from(&self, buf: &mut [u8]) -> io::Result<(usize, SocketAddr)>;

    /// Local address this transport is bound to.
    fn local_addr(&self) -> io::Result<SocketAddr>;

    /// Join a multicast group.
    fn join_multicast(&self, group: &str) -> io::Result<()>;

    /// Raw file descriptor for event loop registration.
    #[cfg(unix)]
    fn raw_fd(&self) -> std::os::unix::io::RawFd;
}
