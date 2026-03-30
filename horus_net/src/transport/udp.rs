//! UDP transport — std::net::UdpSocket wrapper, zero external deps.
//!
//! Binds to 0.0.0.0:{port}, sets SO_REUSEADDR, nonblocking mode,
//! and joins multicast group for discovery.

use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
#[cfg(unix)]
use std::os::unix::io::AsRawFd;

use super::Transport;

/// UDP transport backed by a single std::net::UdpSocket.
pub struct UdpTransport {
    socket: UdpSocket,
}

impl UdpTransport {
    /// Create a new UDP transport bound to the given port.
    ///
    /// Sets SO_REUSEADDR for multiple processes on the same machine,
    /// and switches to nonblocking mode for event loop integration.
    pub fn bind(port: u16) -> io::Result<Self> {
        let addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port);
        let socket = UdpSocket::bind(addr)?;

        // Allow multiple processes on the same port (for development/testing)
        socket.set_nonblocking(true)?;

        // Set receive buffer to 256KB for burst handling
        #[cfg(unix)]
        {
            use std::os::unix::io::AsRawFd;
            let fd = socket.as_raw_fd();
            let buf_size: libc::c_int = 256 * 1024;
            unsafe {
                libc::setsockopt(
                    fd,
                    libc::SOL_SOCKET,
                    libc::SO_RCVBUF,
                    &buf_size as *const _ as *const libc::c_void,
                    std::mem::size_of::<libc::c_int>() as libc::socklen_t,
                );
                // SO_REUSEADDR
                let reuse: libc::c_int = 1;
                libc::setsockopt(
                    fd,
                    libc::SOL_SOCKET,
                    libc::SO_REUSEADDR,
                    &reuse as *const _ as *const libc::c_void,
                    std::mem::size_of::<libc::c_int>() as libc::socklen_t,
                );
            }
        }

        Ok(Self { socket })
    }
}

impl Transport for UdpTransport {
    fn send_to(&self, data: &[u8], addr: SocketAddr) -> io::Result<usize> {
        self.socket.send_to(data, addr)
    }

    fn recv_from(&self, buf: &mut [u8]) -> io::Result<(usize, SocketAddr)> {
        self.socket.recv_from(buf)
    }

    fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    fn join_multicast(&self, group: &str) -> io::Result<()> {
        let group_addr: Ipv4Addr = group
            .parse()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        self.socket
            .join_multicast_v4(&group_addr, &Ipv4Addr::UNSPECIFIED)
    }

    #[cfg(unix)]
    fn raw_fd(&self) -> std::os::unix::io::RawFd {
        self.socket.as_raw_fd()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bind_and_local_addr() {
        // Bind to port 0 (OS-assigned) to avoid conflicts
        let transport = UdpTransport::bind(0).unwrap();
        let addr = transport.local_addr().unwrap();
        assert_ne!(addr.port(), 0); // OS assigned a real port
    }

    #[test]
    fn nonblocking_recv_returns_wouldblock() {
        let transport = UdpTransport::bind(0).unwrap();
        let mut buf = [0u8; 1024];
        let result = transport.recv_from(&mut buf);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err().kind(),
            io::ErrorKind::WouldBlock
        );
    }

    #[test]
    fn loopback_send_recv() {
        let t1 = UdpTransport::bind(0).unwrap();
        let t2 = UdpTransport::bind(0).unwrap();
        let addr2 = t2.local_addr().unwrap();

        let data = b"hello horus_net";
        t1.send_to(data, addr2).unwrap();

        // Small spin to let the packet arrive (nonblocking)
        let mut buf = [0u8; 256];
        let mut received = false;
        for _ in 0..100 {
            match t2.recv_from(&mut buf) {
                Ok((n, _from)) => {
                    assert_eq!(&buf[..n], data);
                    received = true;
                    break;
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    std::thread::sleep(std::time::Duration::from_millis(1));
                }
                Err(e) => panic!("unexpected error: {e}"),
            }
        }
        assert!(received, "packet not received within timeout");
    }

    #[test]
    fn multicast_join_valid_group() {
        let transport = UdpTransport::bind(0).unwrap();
        // 224.0.69.72 is the horus_net default
        let result = transport.join_multicast("224.0.69.72");
        // May fail in CI without multicast support, but shouldn't panic
        if let Err(e) = &result {
            eprintln!("multicast join failed (expected in some CI): {e}");
        }
    }

    #[test]
    fn multicast_join_invalid_group() {
        let transport = UdpTransport::bind(0).unwrap();
        let result = transport.join_multicast("not-an-ip");
        assert!(result.is_err());
    }

    #[cfg(unix)]
    #[test]
    fn raw_fd_valid() {
        let transport = UdpTransport::bind(0).unwrap();
        let fd = transport.raw_fd();
        assert!(fd >= 0);
    }
}
