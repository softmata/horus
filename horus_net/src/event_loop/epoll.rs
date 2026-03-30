//! Linux epoll event loop — UDP fd + eventfd + timerfd.
//!
//! Three event sources:
//! - UDP socket fd: incoming network packets (import path)
//! - eventfd: signaled by TopicRegistry when new SHM data arrives (export path)
//! - timerfd: periodic heartbeat (50ms) and discovery (1s)

use std::io;
use std::os::unix::io::RawFd;
use std::time::Duration;

use super::{Event, EventSource};

/// Token values stored in epoll_event.u64 to identify which fd fired.
const TOKEN_UDP: u64 = 1;
const TOKEN_SHM: u64 = 2;
const TOKEN_TIMER: u64 = 3;

/// Epoll-based event loop for Linux.
pub struct EpollLoop {
    epoll_fd: RawFd,
    event_fd: RawFd,
    timer_fd: RawFd,
    udp_fd: Option<RawFd>,
}

impl EpollLoop {
    /// Create a new epoll event loop with eventfd and timerfd.
    ///
    /// `timer_interval` is the periodic timer interval (typically 50ms for heartbeat).
    pub fn new(timer_interval: Duration) -> io::Result<Self> {
        // Create epoll instance
        let epoll_fd = unsafe { libc::epoll_create1(libc::EPOLL_CLOEXEC) };
        if epoll_fd < 0 {
            return Err(io::Error::last_os_error());
        }

        // Create eventfd for SHM notifications
        let event_fd = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC) };
        if event_fd < 0 {
            unsafe { libc::close(epoll_fd) };
            return Err(io::Error::last_os_error());
        }

        // Create timerfd for periodic tasks
        let timer_fd = unsafe {
            libc::timerfd_create(
                libc::CLOCK_MONOTONIC,
                libc::TFD_NONBLOCK | libc::TFD_CLOEXEC,
            )
        };
        if timer_fd < 0 {
            unsafe {
                libc::close(event_fd);
                libc::close(epoll_fd);
            }
            return Err(io::Error::last_os_error());
        }

        // Arm the timer
        let secs = timer_interval.as_secs() as libc::time_t;
        let nsecs = timer_interval.subsec_nanos() as libc::c_long;
        let timer_spec = libc::itimerspec {
            it_interval: libc::timespec {
                tv_sec: secs,
                tv_nsec: nsecs,
            },
            it_value: libc::timespec {
                tv_sec: secs,
                tv_nsec: nsecs,
            },
        };
        let ret = unsafe { libc::timerfd_settime(timer_fd, 0, &timer_spec, std::ptr::null_mut()) };
        if ret < 0 {
            unsafe {
                libc::close(timer_fd);
                libc::close(event_fd);
                libc::close(epoll_fd);
            }
            return Err(io::Error::last_os_error());
        }

        let mut loop_inst = Self {
            epoll_fd,
            event_fd,
            timer_fd,
            udp_fd: None,
        };

        // Register eventfd and timerfd with epoll
        loop_inst.epoll_add(event_fd, TOKEN_SHM)?;
        loop_inst.epoll_add(timer_fd, TOKEN_TIMER)?;

        Ok(loop_inst)
    }

    /// Register the UDP socket fd with epoll.
    pub fn register_udp(&mut self, fd: RawFd) -> io::Result<()> {
        self.epoll_add(fd, TOKEN_UDP)?;
        self.udp_fd = Some(fd);
        Ok(())
    }

    /// Get the eventfd for SHM notifications.
    /// Write a u64(1) to this fd to wake the event loop.
    pub fn shm_notify_fd(&self) -> RawFd {
        self.event_fd
    }

    /// Signal the SHM eventfd (wake the event loop for export).
    pub fn signal_shm(&self) -> io::Result<()> {
        let val: u64 = 1;
        let ret =
            unsafe { libc::write(self.event_fd, &val as *const u64 as *const libc::c_void, 8) };
        if ret < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// Block until events arrive. Returns the events that fired.
    ///
    /// `timeout_ms`: -1 for infinite wait, 0 for poll, >0 for timeout.
    pub fn wait(&self, timeout_ms: i32) -> io::Result<Vec<Event>> {
        let mut raw_events = [libc::epoll_event { events: 0, u64: 0 }; 8];

        let n = unsafe {
            libc::epoll_wait(
                self.epoll_fd,
                raw_events.as_mut_ptr(),
                raw_events.len() as i32,
                timeout_ms,
            )
        };

        if n < 0 {
            let err = io::Error::last_os_error();
            // EINTR is normal (signal interrupted) — return empty
            if err.raw_os_error() == Some(libc::EINTR) {
                return Ok(Vec::new());
            }
            return Err(err);
        }

        let mut events = Vec::with_capacity(n as usize);
        for i in 0..n as usize {
            let source = match raw_events[i].u64 {
                TOKEN_UDP => EventSource::UdpSocket,
                TOKEN_SHM => {
                    // Consume the eventfd counter to re-arm it
                    let mut val: u64 = 0;
                    unsafe {
                        libc::read(self.event_fd, &mut val as *mut u64 as *mut libc::c_void, 8);
                    }
                    EventSource::ShmNotify
                }
                TOKEN_TIMER => {
                    // Consume the timerfd expiration count to re-arm it
                    let mut val: u64 = 0;
                    unsafe {
                        libc::read(self.timer_fd, &mut val as *mut u64 as *mut libc::c_void, 8);
                    }
                    EventSource::Timer
                }
                _ => continue,
            };
            events.push(Event { source });
        }

        Ok(events)
    }

    fn epoll_add(&self, fd: RawFd, token: u64) -> io::Result<()> {
        let mut ev = libc::epoll_event {
            events: (libc::EPOLLIN | libc::EPOLLET) as u32,
            u64: token,
        };
        let ret = unsafe { libc::epoll_ctl(self.epoll_fd, libc::EPOLL_CTL_ADD, fd, &mut ev) };
        if ret < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
}

impl Drop for EpollLoop {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.timer_fd);
            libc::close(self.event_fd);
            libc::close(self.epoll_fd);
            // Note: UDP fd is NOT closed here — it's owned by UdpTransport
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::io::AsRawFd;

    #[test]
    fn create_event_loop() {
        let el = EpollLoop::new(Duration::from_millis(50)).unwrap();
        assert!(el.epoll_fd >= 0);
        assert!(el.event_fd >= 0);
        assert!(el.timer_fd >= 0);
    }

    #[test]
    fn eventfd_wakeup() {
        let el = EpollLoop::new(Duration::from_secs(60)).unwrap(); // Long timer so it won't fire
        el.signal_shm().unwrap();

        let events = el.wait(100).unwrap(); // 100ms timeout
        assert!(!events.is_empty());
        assert_eq!(events[0].source, EventSource::ShmNotify);
    }

    #[test]
    fn timer_fires() {
        let el = EpollLoop::new(Duration::from_millis(10)).unwrap(); // 10ms timer

        // Wait up to 100ms for timer to fire
        let events = el.wait(100).unwrap();
        assert!(!events.is_empty());
        assert!(events.iter().any(|e| e.source == EventSource::Timer));
    }

    #[test]
    fn no_events_returns_empty() {
        let el = EpollLoop::new(Duration::from_secs(60)).unwrap();

        // Poll with 0 timeout — should return immediately with no events
        let events = el.wait(0).unwrap();
        assert!(events.is_empty());
    }

    #[test]
    fn udp_registration() {
        let socket = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
        socket.set_nonblocking(true).unwrap();
        let fd = socket.as_raw_fd();

        let mut el = EpollLoop::new(Duration::from_secs(60)).unwrap();
        el.register_udp(fd).unwrap();
        assert_eq!(el.udp_fd, Some(fd));
    }

    #[test]
    fn udp_data_wakeup() {
        let recv_socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        recv_socket.set_nonblocking(true).unwrap();
        let recv_addr = recv_socket.local_addr().unwrap();
        let recv_fd = recv_socket.as_raw_fd();

        let mut el = EpollLoop::new(Duration::from_secs(60)).unwrap();
        el.register_udp(recv_fd).unwrap();

        // Send data from another socket
        let send_socket = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
        send_socket.send_to(b"test", recv_addr).unwrap();

        // epoll should wake up with UdpSocket event
        let events = el.wait(100).unwrap();
        assert!(!events.is_empty());
        assert!(events.iter().any(|e| e.source == EventSource::UdpSocket));
    }

    #[test]
    fn fds_closed_on_drop() {
        let (epoll_fd, event_fd, timer_fd);
        {
            let el = EpollLoop::new(Duration::from_millis(50)).unwrap();
            epoll_fd = el.epoll_fd;
            event_fd = el.event_fd;
            timer_fd = el.timer_fd;
        }
        // After drop, fds should be closed. Attempting to use them should fail.
        // We can verify by checking fcntl returns -1/EBADF
        unsafe {
            assert_eq!(libc::fcntl(epoll_fd, libc::F_GETFD), -1);
            assert_eq!(libc::fcntl(event_fd, libc::F_GETFD), -1);
            assert_eq!(libc::fcntl(timer_fd, libc::F_GETFD), -1);
        }
    }
}
