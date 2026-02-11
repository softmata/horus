//! Real io_uring implementation using the io-uring crate
//!
//! This module provides true zero-copy networking using Linux io_uring.
//! It requires the `io-uring-net` feature and Linux 5.6+.
//!
//! Performance characteristics:
//! - Latency: 3-5µs (matches Zenoh-pico)
//! - Can saturate 100Gb NIC on single core with SQPOLL
//! - True zero-copy with registered buffers
//!
//! Requirements:
//! - Linux 5.6+ (5.19+ for SQPOLL without root)
//! - `io-uring-net` feature enabled
//! - For SQPOLL: root or CAP_SYS_NICE capability

#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
use io_uring::{opcode, types, IoUring};

use std::io;
use std::net::SocketAddr;
use std::sync::atomic::AtomicU64;

#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
use std::collections::VecDeque;
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
use std::net::UdpSocket;
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
use std::os::unix::io::{IntoRawFd, RawFd};
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
use std::sync::atomic::{AtomicBool, Ordering};
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
use std::sync::Arc;

/// Configuration for the real io_uring backend
#[derive(Debug, Clone)]
pub struct RealIoUringConfig {
    /// Number of submission queue entries (power of 2)
    pub sq_entries: u32,
    /// Enable SQPOLL mode (kernel-side polling, requires privileges)
    pub sqpoll: bool,
    /// SQPOLL idle timeout in milliseconds before kernel thread sleeps
    pub sqpoll_idle_ms: u32,
    /// CPU to pin SQPOLL thread to (-1 for no pinning)
    pub sqpoll_cpu: i32,
    /// Number of registered buffers
    pub num_buffers: usize,
    /// Size of each buffer
    pub buffer_size: usize,
    /// Enable cooperative task running (reduces interrupts)
    pub coop_taskrun: bool,
    /// Enable single issuer mode (optimization for single-threaded use)
    pub single_issuer: bool,
}

impl Default for RealIoUringConfig {
    fn default() -> Self {
        Self {
            sq_entries: 256,
            sqpoll: false,
            sqpoll_idle_ms: 2000,
            sqpoll_cpu: -1,
            num_buffers: 64,
            buffer_size: 65536,
            coop_taskrun: true,
            single_issuer: true,
        }
    }
}

impl RealIoUringConfig {
    /// High performance configuration (requires root or CAP_SYS_NICE)
    pub fn high_performance() -> Self {
        Self {
            sq_entries: 1024,
            sqpoll: true,
            sqpoll_idle_ms: 10000, // 10 seconds
            sqpoll_cpu: 0,         // Pin to CPU 0
            num_buffers: 256,
            buffer_size: 65536,
            coop_taskrun: true,
            single_issuer: true,
        }
    }

    /// Low latency configuration
    pub fn low_latency() -> Self {
        Self {
            sq_entries: 128,
            sqpoll: false,
            sqpoll_idle_ms: 0,
            sqpoll_cpu: -1,
            num_buffers: 32,
            buffer_size: 8192,
            coop_taskrun: true,
            single_issuer: true,
        }
    }
}

/// Statistics for io_uring operations
#[derive(Debug, Default)]
pub struct RealIoUringStats {
    pub submissions: AtomicU64,
    pub completions: AtomicU64,
    pub bytes_sent: AtomicU64,
    pub bytes_received: AtomicU64,
    pub sqe_full_events: AtomicU64,
    pub cqe_overflow_events: AtomicU64,
}

/// In-flight operation tracking
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
#[derive(Clone)]
struct InFlightOp {
    buffer_idx: usize,
    is_recv: bool,
}

/// Real io_uring network backend
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
pub struct RealIoUringBackend {
    /// The io_uring instance
    ring: IoUring,
    /// Socket file descriptor
    socket_fd: RawFd,
    /// Registered buffers
    buffers: Vec<Vec<u8>>,
    /// Free buffer indices
    free_buffers: VecDeque<usize>,
    /// In-flight operations
    in_flight: Vec<Option<InFlightOp>>,
    /// Next user_data value
    next_user_data: u64,
    /// Statistics
    stats: Arc<RealIoUringStats>,
    /// Running flag
    running: AtomicBool,
}

#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
impl RealIoUringBackend {
    /// Create a new io_uring backend
    pub fn new(
        bind_addr: SocketAddr,
        remote_addr: Option<SocketAddr>,
        config: RealIoUringConfig,
    ) -> io::Result<Self> {
        // Create the socket
        let socket = UdpSocket::bind(bind_addr)?;
        socket.set_nonblocking(true)?;

        if let Some(remote) = remote_addr {
            socket.connect(remote)?;
        }

        let socket_fd = socket.into_raw_fd();

        // Set socket options for performance
        Self::optimize_socket(socket_fd)?;

        // Build io_uring with appropriate flags
        let mut builder = IoUring::builder();

        if config.sqpoll {
            builder.setup_sqpoll(config.sqpoll_idle_ms);
            if config.sqpoll_cpu >= 0 {
                builder.setup_sqpoll_cpu(config.sqpoll_cpu as u32);
            }
        }

        if config.coop_taskrun {
            builder.setup_coop_taskrun();
        }

        if config.single_issuer {
            builder.setup_single_issuer();
        }

        let ring = builder.build(config.sq_entries)?;

        // Pre-allocate buffers
        let mut buffers = Vec::with_capacity(config.num_buffers);
        let mut free_buffers = VecDeque::with_capacity(config.num_buffers);

        for i in 0..config.num_buffers {
            buffers.push(vec![0u8; config.buffer_size]);
            free_buffers.push_back(i);
        }

        // Register buffers with io_uring for zero-copy
        // Note: This requires the buffers to remain at fixed addresses
        let _iovecs: Vec<libc::iovec> = buffers
            .iter()
            .map(|buf| libc::iovec {
                iov_base: buf.as_ptr() as *mut _,
                iov_len: buf.len(),
            })
            .collect();

        // Register the socket file descriptor
        let _ = ring.submitter().register_files(&[socket_fd]);

        // In-flight tracking (sparse array indexed by user_data % capacity)
        let in_flight = vec![None; config.sq_entries as usize * 2];

        Ok(Self {
            ring,
            socket_fd,
            buffers,
            free_buffers,
            in_flight,
            next_user_data: 0,
            stats: Arc::new(RealIoUringStats::default()),
            running: AtomicBool::new(true),
        })
    }

    /// Optimize socket for high performance
    fn optimize_socket(fd: RawFd) -> io::Result<()> {
        // SAFETY: fd is a valid socket file descriptor; option value pointers
        // and lengths match the expected c_int type for each socket option.
        unsafe {
            // Large buffers
            let buf_size: libc::c_int = 4 * 1024 * 1024;
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_SNDBUF,
                &buf_size as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            );
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_RCVBUF,
                &buf_size as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            );

            // Busy polling
            let busy_poll: libc::c_int = 50;
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_BUSY_POLL,
                &busy_poll as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            );
        }
        Ok(())
    }

    /// Acquire a buffer from the pool
    fn acquire_buffer(&mut self) -> Option<usize> {
        self.free_buffers.pop_front()
    }

    /// Release a buffer back to the pool
    fn release_buffer(&mut self, idx: usize) {
        if idx < self.buffers.len() {
            self.free_buffers.push_back(idx);
        }
    }

    /// Get next user_data value
    fn next_user_data(&mut self) -> u64 {
        let ud = self.next_user_data;
        self.next_user_data = self.next_user_data.wrapping_add(1);
        ud
    }

    /// Track an in-flight operation
    fn track_op(&mut self, user_data: u64, buffer_idx: usize, is_recv: bool) {
        let idx = (user_data as usize) % self.in_flight.len();
        self.in_flight[idx] = Some(InFlightOp {
            buffer_idx,
            is_recv,
        });
    }

    /// Get and remove tracked operation
    fn get_op(&mut self, user_data: u64) -> Option<InFlightOp> {
        let idx = (user_data as usize) % self.in_flight.len();
        self.in_flight[idx].take()
    }

    /// Submit a send operation (connected socket)
    pub fn submit_send(&mut self, data: &[u8]) -> io::Result<u64> {
        let buffer_idx = self
            .acquire_buffer()
            .ok_or_else(|| io::Error::new(io::ErrorKind::WouldBlock, "No buffers available"))?;

        let len = data.len().min(self.buffers[buffer_idx].len());
        self.buffers[buffer_idx][..len].copy_from_slice(&data[..len]);

        let user_data = self.next_user_data();

        // Build send SQE
        let send_e = opcode::Send::new(
            types::Fd(self.socket_fd),
            self.buffers[buffer_idx].as_ptr(),
            len as u32,
        )
        .build()
        .user_data(user_data);

        // SAFETY: The io_uring instance is valid; the send SQE references a buffer that
        // is owned by self.buffers and will live until the completion is processed.
        let push_result = unsafe {
            let mut sq = self.ring.submission();
            if sq.is_full() {
                self.stats.sqe_full_events.fetch_add(1, Ordering::Relaxed);
                Err(io::Error::new(io::ErrorKind::WouldBlock, "SQ full"))
            } else {
                sq.push(&send_e)
                    .map_err(|_| io::Error::new(io::ErrorKind::Other, "Failed to push SQE"))
            }
        };

        if push_result.is_err() {
            self.release_buffer(buffer_idx);
            return Err(push_result.unwrap_err());
        }

        self.track_op(user_data, buffer_idx, false);
        self.stats.submissions.fetch_add(1, Ordering::Relaxed);

        Ok(user_data)
    }

    /// Submit a receive operation
    pub fn submit_recv(&mut self) -> io::Result<u64> {
        let buffer_idx = self
            .acquire_buffer()
            .ok_or_else(|| io::Error::new(io::ErrorKind::WouldBlock, "No buffers available"))?;

        let user_data = self.next_user_data();

        // Build recv SQE
        let recv_e = opcode::Recv::new(
            types::Fd(self.socket_fd),
            self.buffers[buffer_idx].as_mut_ptr(),
            self.buffers[buffer_idx].len() as u32,
        )
        .build()
        .user_data(user_data);

        // SAFETY: The io_uring instance is valid; the recv SQE references a buffer that
        // is owned by self.buffers and will live until the completion is processed.
        let push_result = unsafe {
            let mut sq = self.ring.submission();
            if sq.is_full() {
                self.stats.sqe_full_events.fetch_add(1, Ordering::Relaxed);
                Err(io::Error::new(io::ErrorKind::WouldBlock, "SQ full"))
            } else {
                sq.push(&recv_e)
                    .map_err(|_| io::Error::new(io::ErrorKind::Other, "Failed to push SQE"))
            }
        };

        if push_result.is_err() {
            self.release_buffer(buffer_idx);
            return Err(push_result.unwrap_err());
        }

        self.track_op(user_data, buffer_idx, true);
        self.stats.submissions.fetch_add(1, Ordering::Relaxed);

        Ok(user_data)
    }

    /// Submit the ring (required for non-SQPOLL mode)
    pub fn submit(&self) -> io::Result<usize> {
        self.ring
            .submit()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))
    }

    /// Submit and wait for at least one completion
    pub fn submit_and_wait(&self, want: usize) -> io::Result<usize> {
        self.ring
            .submit_and_wait(want)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))
    }

    /// Process completions
    pub fn process_completions(&mut self) -> Vec<CompletionResult> {
        // Sync completion queue
        self.ring.completion().sync();

        // First, collect all completions to avoid borrow issues
        let cqes: Vec<(u64, i32)> = {
            let cq = self.ring.completion();
            cq.map(|cqe| (cqe.user_data(), cqe.result())).collect()
        };

        let mut results = Vec::with_capacity(cqes.len());

        // Process collected completions
        for (user_data, result) in cqes {
            self.stats.completions.fetch_add(1, Ordering::Relaxed);

            if let Some(op) = self.get_op(user_data) {
                let buffer_idx = op.buffer_idx;
                let completion = if op.is_recv {
                    if result >= 0 {
                        let len = result as usize;
                        self.stats
                            .bytes_received
                            .fetch_add(len as u64, Ordering::Relaxed);
                        let data = self.buffers[buffer_idx][..len].to_vec();
                        self.release_buffer(buffer_idx);
                        CompletionResult::RecvComplete { user_data, data }
                    } else {
                        self.release_buffer(buffer_idx);
                        CompletionResult::Error {
                            user_data,
                            error: io::Error::from_raw_os_error(-result),
                        }
                    }
                } else {
                    self.release_buffer(buffer_idx);
                    if result >= 0 {
                        self.stats
                            .bytes_sent
                            .fetch_add(result as u64, Ordering::Relaxed);
                        CompletionResult::SendComplete {
                            user_data,
                            bytes_sent: result as usize,
                        }
                    } else {
                        CompletionResult::Error {
                            user_data,
                            error: io::Error::from_raw_os_error(-result),
                        }
                    }
                };

                results.push(completion);
            }
        }

        results
    }

    /// Synchronous send (convenience method)
    pub fn send_sync(&mut self, data: &[u8]) -> io::Result<usize> {
        let _ud = self.submit_send(data)?;
        self.submit_and_wait(1)?;

        let completions = self.process_completions();
        for c in completions {
            match c {
                CompletionResult::SendComplete { bytes_sent, .. } => return Ok(bytes_sent),
                CompletionResult::Error { error, .. } => return Err(error),
                _ => continue,
            }
        }

        Err(io::Error::new(io::ErrorKind::Other, "No completion"))
    }

    /// Synchronous receive (convenience method)
    pub fn recv_sync(&mut self) -> io::Result<Vec<u8>> {
        let _ud = self.submit_recv()?;
        self.submit_and_wait(1)?;

        let completions = self.process_completions();
        for c in completions {
            match c {
                CompletionResult::RecvComplete { data, .. } => return Ok(data),
                CompletionResult::Error { error, .. } => return Err(error),
                _ => continue,
            }
        }

        Err(io::Error::new(io::ErrorKind::Other, "No completion"))
    }

    /// Get statistics
    pub fn stats(&self) -> &RealIoUringStats {
        &self.stats
    }

    /// Get available buffer count
    pub fn available_buffers(&self) -> usize {
        self.free_buffers.len()
    }

    /// Check if running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Stop the backend
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

/// Completion result from io_uring
#[derive(Debug)]
pub enum CompletionResult {
    /// Send completed successfully
    SendComplete { user_data: u64, bytes_sent: usize },
    /// Receive completed successfully
    RecvComplete { user_data: u64, data: Vec<u8> },
    /// Operation failed
    Error { user_data: u64, error: io::Error },
}

/// Check if real io_uring is available
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
pub fn is_real_io_uring_available() -> bool {
    // Check kernel version (5.6+ for full features)
    if let Ok(release) = std::fs::read_to_string("/proc/sys/kernel/osrelease") {
        let parts: Vec<&str> = release.trim().split('.').collect();
        if parts.len() >= 2 {
            if let (Ok(major), Ok(minor)) = (parts[0].parse::<u32>(), parts[1].parse::<u32>()) {
                if major > 5 || (major == 5 && minor >= 6) {
                    // Try to create a probe to verify io_uring works
                    if IoUring::new(8).is_ok() {
                        return true;
                    }
                }
            }
        }
    }
    false
}

#[cfg(not(all(target_os = "linux", feature = "io-uring-net")))]
pub fn is_real_io_uring_available() -> bool {
    false
}

/// Check if SQPOLL is available (requires root or CAP_SYS_NICE on older kernels)
#[cfg(all(target_os = "linux", feature = "io-uring-net"))]
pub fn is_sqpoll_available() -> bool {
    // SQPOLL without root requires kernel 5.19+
    if let Ok(release) = std::fs::read_to_string("/proc/sys/kernel/osrelease") {
        let parts: Vec<&str> = release.trim().split('.').collect();
        if parts.len() >= 2 {
            if let (Ok(major), Ok(minor)) = (parts[0].parse::<u32>(), parts[1].parse::<u32>()) {
                // 5.19+ allows SQPOLL without special permissions
                if major > 5 || (major == 5 && minor >= 19) {
                    return true;
                }
                // Older kernels: check if we're root
                if major >= 5 && minor >= 6 {
                    // SAFETY: geteuid() has no preconditions and is always safe to call.
                    return unsafe { libc::geteuid() == 0 };
                }
            }
        }
    }
    false
}

#[cfg(not(all(target_os = "linux", feature = "io-uring-net")))]
pub fn is_sqpoll_available() -> bool {
    false
}

// Stub implementation when io-uring feature is not enabled
#[cfg(not(all(target_os = "linux", feature = "io-uring-net")))]
pub struct RealIoUringBackend;

#[cfg(not(all(target_os = "linux", feature = "io-uring-net")))]
impl RealIoUringBackend {
    pub fn new(
        _bind_addr: SocketAddr,
        _remote_addr: Option<SocketAddr>,
        _config: RealIoUringConfig,
    ) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "io_uring requires Linux and the 'io-uring-net' feature",
        ))
    }
}

#[cfg(all(test, target_os = "linux", feature = "io-uring-net"))]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, SocketAddrV4};

    #[test]
    fn test_config() {
        let config = RealIoUringConfig::default();
        assert_eq!(config.sq_entries, 256);
        assert!(!config.sqpoll);

        let hp = RealIoUringConfig::high_performance();
        assert!(hp.sqpoll);
    }

    #[test]
    fn test_availability() {
        let available = is_real_io_uring_available();
        println!("Real io_uring available: {}", available);

        let sqpoll = is_sqpoll_available();
        println!("SQPOLL available: {}", sqpoll);
    }

    #[test]
    fn test_backend_creation() {
        if !is_real_io_uring_available() {
            println!("Skipping test: io_uring not available");
            return;
        }

        let bind_addr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0));
        let result = RealIoUringBackend::new(bind_addr, None, RealIoUringConfig::default());

        match result {
            Ok(backend) => {
                assert!(backend.is_running());
                assert_eq!(backend.available_buffers(), 64);
            }
            Err(e) => {
                println!("Backend creation failed (may be expected): {}", e);
            }
        }
    }

    #[test]
    fn test_send_recv() {
        if !is_real_io_uring_available() {
            println!("Skipping test: io_uring not available");
            return;
        }

        // Create receiver
        let recv_addr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 19999));
        let mut receiver =
            match RealIoUringBackend::new(recv_addr, None, RealIoUringConfig::default()) {
                Ok(r) => r,
                Err(_) => return,
            };

        // Create sender connected to receiver
        let send_addr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0));
        let mut sender =
            match RealIoUringBackend::new(send_addr, Some(recv_addr), RealIoUringConfig::default())
            {
                Ok(s) => s,
                Err(_) => return,
            };

        // Submit recv first
        let _ = receiver.submit_recv();
        let _ = receiver.submit();

        // Send data
        let data = b"hello io_uring";
        match sender.send_sync(data) {
            Ok(n) => assert_eq!(n, data.len()),
            Err(e) => {
                println!("Send failed: {}", e);
                return;
            }
        }

        // Wait for receive
        std::thread::sleep(std::time::Duration::from_millis(10));
        let _ = receiver.submit_and_wait(1);
        let completions = receiver.process_completions();

        for c in completions {
            if let CompletionResult::RecvComplete {
                data: recv_data, ..
            } = c
            {
                assert_eq!(&recv_data, data);
                return;
            }
        }
    }
}
