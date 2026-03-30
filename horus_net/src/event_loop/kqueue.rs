//! macOS kqueue event loop — UDP fd + pipe + EVFILT_TIMER.
//!
//! macOS has no eventfd, so we use a pipe pair for SHM notifications.
//! Same EventSource/Event interface as the epoll implementation.

#![cfg(target_os = "macos")]

use std::io;
use std::os::unix::io::RawFd;
use std::time::Duration;

use super::{Event, EventSource};

/// Identifier values for kqueue events.
const IDENT_UDP: usize = 1;
const IDENT_SHM: usize = 2;
const IDENT_TIMER: usize = 3;

/// Kqueue-based event loop for macOS.
pub struct KqueueLoop {
    kq: RawFd,
    /// Pipe for SHM notifications: write end signals, read end registered with kqueue.
    pipe_read: RawFd,
    pipe_write: RawFd,
    udp_fd: Option<RawFd>,
}

impl KqueueLoop {
    /// Create a new kqueue event loop with pipe notification and timer.
    ///
    /// `timer_interval` is the periodic timer interval (typically 50ms for heartbeat).
    pub fn new(timer_interval: Duration) -> io::Result<Self> {
        let kq = unsafe { libc::kqueue() };
        if kq < 0 {
            return Err(io::Error::last_os_error());
        }

        // Create pipe for SHM notifications (replaces eventfd)
        let mut pipe_fds = [0i32; 2];
        if unsafe { libc::pipe(pipe_fds.as_mut_ptr()) } < 0 {
            unsafe { libc::close(kq) };
            return Err(io::Error::last_os_error());
        }
        let pipe_read = pipe_fds[0];
        let pipe_write = pipe_fds[1];

        // Set pipe read end to nonblocking
        unsafe {
            let flags = libc::fcntl(pipe_read, libc::F_GETFL);
            libc::fcntl(pipe_read, libc::F_SETFL, flags | libc::O_NONBLOCK);
            let flags = libc::fcntl(pipe_write, libc::F_GETFL);
            libc::fcntl(pipe_write, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }

        // Register pipe read fd with kqueue
        let timer_ms = timer_interval.as_millis() as isize;

        let changes = [
            // Pipe read fd (SHM notification)
            libc::kevent {
                ident: IDENT_SHM,
                filter: libc::EVFILT_READ,
                flags: libc::EV_ADD | libc::EV_ENABLE,
                fflags: 0,
                data: 0,
                udata: std::ptr::null_mut(),
            },
            // Timer
            libc::kevent {
                ident: IDENT_TIMER,
                filter: libc::EVFILT_TIMER,
                flags: libc::EV_ADD | libc::EV_ENABLE,
                fflags: 0,
                data: timer_ms,
                udata: std::ptr::null_mut(),
            },
        ];

        // The pipe read fd needs to be in the ident field for EVFILT_READ
        let mut actual_changes = changes;
        actual_changes[0].ident = pipe_read as usize;

        let ret = unsafe {
            libc::kevent(
                kq,
                actual_changes.as_ptr(),
                actual_changes.len() as i32,
                std::ptr::null_mut(),
                0,
                std::ptr::null(),
            )
        };
        if ret < 0 {
            unsafe {
                libc::close(pipe_write);
                libc::close(pipe_read);
                libc::close(kq);
            }
            return Err(io::Error::last_os_error());
        }

        Ok(Self {
            kq,
            pipe_read,
            pipe_write,
            udp_fd: None,
        })
    }

    /// Register the UDP socket fd with kqueue.
    pub fn register_udp(&mut self, fd: RawFd) -> io::Result<()> {
        let change = libc::kevent {
            ident: fd as usize,
            filter: libc::EVFILT_READ,
            flags: libc::EV_ADD | libc::EV_ENABLE,
            fflags: 0,
            data: 0,
            udata: std::ptr::null_mut(),
        };
        let ret = unsafe {
            libc::kevent(self.kq, &change, 1, std::ptr::null_mut(), 0, std::ptr::null())
        };
        if ret < 0 {
            return Err(io::Error::last_os_error());
        }
        self.udp_fd = Some(fd);
        Ok(())
    }

    /// Get the pipe write fd for SHM notifications.
    /// Write a byte to this fd to wake the event loop.
    pub fn shm_notify_fd(&self) -> RawFd {
        self.pipe_write
    }

    /// Signal the SHM pipe (wake the event loop for export).
    pub fn signal_shm(&self) -> io::Result<()> {
        let byte: u8 = 1;
        let ret = unsafe {
            libc::write(self.pipe_write, &byte as *const u8 as *const libc::c_void, 1)
        };
        if ret < 0 {
            let err = io::Error::last_os_error();
            // EAGAIN is fine — pipe already has data, event loop will wake
            if err.raw_os_error() == Some(libc::EAGAIN) {
                return Ok(());
            }
            Err(err)
        } else {
            Ok(())
        }
    }

    /// Block until events arrive. Returns the events that fired.
    pub fn wait(&self, timeout_ms: i32) -> io::Result<Vec<Event>> {
        let mut raw_events = [libc::kevent {
            ident: 0,
            filter: 0,
            flags: 0,
            fflags: 0,
            data: 0,
            udata: std::ptr::null_mut(),
        }; 8];

        let timeout = if timeout_ms < 0 {
            std::ptr::null()
        } else {
            &libc::timespec {
                tv_sec: (timeout_ms / 1000) as libc::time_t,
                tv_nsec: ((timeout_ms % 1000) * 1_000_000) as libc::c_long,
            }
        };

        let n = unsafe {
            libc::kevent(
                self.kq,
                std::ptr::null(),
                0,
                raw_events.as_mut_ptr(),
                raw_events.len() as i32,
                timeout,
            )
        };

        if n < 0 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EINTR) {
                return Ok(Vec::new());
            }
            return Err(err);
        }

        let mut events = Vec::with_capacity(n as usize);
        for i in 0..n as usize {
            let ev = &raw_events[i];
            let source = if ev.filter == libc::EVFILT_TIMER {
                EventSource::Timer
            } else if ev.ident == self.pipe_read as usize {
                // Drain the pipe to re-arm
                let mut drain = [0u8; 64];
                while unsafe {
                    libc::read(self.pipe_read, drain.as_mut_ptr() as *mut libc::c_void, drain.len())
                } > 0 {}
                EventSource::ShmNotify
            } else if self.udp_fd == Some(ev.ident as RawFd) {
                EventSource::UdpSocket
            } else {
                continue;
            };
            events.push(Event { source });
        }

        Ok(events)
    }
}

impl Drop for KqueueLoop {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.pipe_write);
            libc::close(self.pipe_read);
            libc::close(self.kq);
            // Note: UDP fd is NOT closed here — owned by UdpTransport
        }
    }
}
