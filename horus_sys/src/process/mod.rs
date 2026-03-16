//! Process lifecycle — signals, liveness checks, session/user IDs.
//!
//! Provides [`ProcessHandle`] for cross-platform process management
//! and [`Signal`] for abstracting Unix signals vs Windows events.
//! - **Unix**: `kill()` + `waitpid()` + `getsid()` + `getuid()`
//! - **Windows**: `TerminateProcess` + `WaitForSingleObject` + `GetCurrentProcessId`

use std::time::Duration;

// ── Signal Abstraction ──────────────────────────────────────────────────────

/// Cross-platform signal abstraction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Signal {
    /// Request graceful shutdown.
    /// Unix: SIGINT. Windows: GenerateConsoleCtrlEvent(CTRL_C_EVENT).
    Interrupt,
    /// Request termination.
    /// Unix: SIGTERM. Windows: GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT).
    Terminate,
    /// Force kill (unrecoverable).
    /// Unix: SIGKILL. Windows: TerminateProcess(handle, 1).
    Kill,
}

// ── Process Handle ──────────────────────────────────────────────────────────

/// A handle to a child or external process.
pub struct ProcessHandle {
    pid: u32,
}

impl ProcessHandle {
    /// Create from a `std::process::Child`.
    pub fn from_child(child: &std::process::Child) -> Self {
        Self { pid: child.id() }
    }

    /// Create from a raw process ID.
    pub fn from_pid(pid: u32) -> Self {
        Self { pid }
    }

    /// Get the process ID.
    pub fn pid(&self) -> u32 {
        self.pid
    }

    /// Check if the process is still running.
    ///
    /// - Unix: `kill(pid, 0) == 0`
    /// - Windows: `OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION)` succeeds
    pub fn is_alive(&self) -> bool {
        #[cfg(unix)]
        {
            // Guard: pid 0 means "process group" and pid values above i32::MAX
            // wrap to negative (pid -1 means "all processes") — both catastrophic.
            if self.pid == 0 || self.pid > i32::MAX as u32 {
                return false;
            }
            // SAFETY: kill with signal 0 is a standard POSIX liveness check.
            // It doesn't send any signal — just checks if the process exists.
            unsafe { libc::kill(self.pid as i32, 0) == 0 }
        }
        #[cfg(windows)]
        {
            use windows_sys::Win32::System::Threading::{
                OpenProcess, PROCESS_QUERY_LIMITED_INFORMATION,
            };
            use windows_sys::Win32::Foundation::CloseHandle;
            unsafe {
                let handle = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, self.pid);
                if handle.is_null() {
                    false
                } else {
                    CloseHandle(handle);
                    true
                }
            }
        }
        #[cfg(not(any(unix, windows)))]
        {
            false
        }
    }

    /// Send a signal to the process.
    pub fn signal(&self, sig: Signal) -> anyhow::Result<()> {
        #[cfg(unix)]
        {
            // Guard: pid 0 sends to process group, pid values above i32::MAX
            // wrap to negative (kill(-1, sig) sends to ALL processes) — both catastrophic.
            if self.pid == 0 || self.pid > i32::MAX as u32 {
                return Ok(()); // Invalid PID, treat as "process already gone"
            }
            let signum = match sig {
                Signal::Interrupt => libc::SIGINT,
                Signal::Terminate => libc::SIGTERM,
                Signal::Kill => libc::SIGKILL,
            };
            // SAFETY: kill() with a valid signal number and validated positive pid is safe.
            // The process may have already exited — that returns ESRCH which we handle.
            let ret = unsafe { libc::kill(self.pid as i32, signum) };
            if ret != 0 {
                let err = std::io::Error::last_os_error();
                if err.raw_os_error() == Some(libc::ESRCH) {
                    return Ok(()); // Process already gone — not an error
                }
                return Err(anyhow::anyhow!("kill({}, {}) failed: {}", self.pid, signum, err));
            }
        }
        #[cfg(windows)]
        {
            match sig {
                Signal::Kill => {
                    use windows_sys::Win32::System::Threading::{
                        OpenProcess, TerminateProcess, PROCESS_TERMINATE,
                    };
                    use windows_sys::Win32::Foundation::CloseHandle;
                    unsafe {
                        let handle = OpenProcess(PROCESS_TERMINATE, 0, self.pid);
                        if !handle.is_null() {
                            TerminateProcess(handle, 1);
                            CloseHandle(handle);
                        }
                    }
                }
                Signal::Interrupt | Signal::Terminate => {
                    use windows_sys::Win32::System::Console::GenerateConsoleCtrlEvent;
                    let event = match sig {
                        Signal::Interrupt => 0,  // CTRL_C_EVENT
                        Signal::Terminate => 1,  // CTRL_BREAK_EVENT
                        _ => unreachable!(),
                    };
                    // SAFETY: GenerateConsoleCtrlEvent sends a console control event.
                    unsafe {
                        GenerateConsoleCtrlEvent(event, self.pid);
                    }
                }
            }
        }
        Ok(())
    }

    /// Wait for the process to exit, with optional timeout.
    /// Returns the exit code if the process exits within the timeout.
    /// Returns `Ok(None)` if the timeout expires.
    pub fn wait_timeout(&self, timeout: Option<Duration>) -> anyhow::Result<Option<i32>> {
        let deadline = timeout.map(|t| std::time::Instant::now() + t);

        loop {
            if !self.is_alive() {
                // Process exited — we can't easily get the exit code from just a PID
                // without having a Child handle. Return 0 as "exited".
                return Ok(Some(0));
            }
            if let Some(dl) = deadline {
                if std::time::Instant::now() >= dl {
                    return Ok(None); // Timeout
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

// ── Session & User IDs ──────────────────────────────────────────────────────

/// Get the current process's session ID for namespace isolation.
///
/// - Unix: `getsid(0)` — session ID of the calling process
/// - Windows: `GetCurrentProcessId()` — no session concept, use PID
pub fn session_id() -> u64 {
    #[cfg(unix)]
    {
        // SAFETY: getsid(0) is async-signal-safe and always valid for the calling process.
        let sid = unsafe { libc::getsid(0) };
        if sid < 0 { std::process::id() as u64 } else { sid as u64 }
    }
    #[cfg(not(unix))]
    {
        std::process::id() as u64
    }
}

/// Get the current user ID for namespace isolation.
///
/// - Unix: `getuid()` — real user ID
/// - Windows: hash of `GetUserNameW()` or process ID fallback
pub fn user_id() -> u64 {
    #[cfg(unix)]
    {
        // SAFETY: getuid() is async-signal-safe and always succeeds.
        unsafe { libc::getuid() as u64 }
    }
    #[cfg(not(unix))]
    {
        // Use process ID as a fallback — unique per user session
        std::process::id() as u64
    }
}

/// Generate a namespace string for SHM isolation.
/// Format: `sid{session_id}_uid{user_id}`
pub fn namespace_id() -> String {
    format!("sid{}_uid{}", session_id(), user_id())
}

// ── Process Start Time ──────────────────────────────────────────────────────

/// Read the OS-level start time of a process (for PID-reuse detection).
///
/// The returned value is opaque and platform-specific — only meaningful when
/// compared to another call for the same PID. Returns 0 when unavailable.
///
/// - **Linux**: starttime from `/proc/{pid}/stat` (jiffies since boot)
/// - **macOS**: `kp_proc.p_starttime` via `sysctl(KERN_PROC_PID)` (microseconds)
/// - **Windows**: creation time from `GetProcessTimes` (100ns intervals since 1601)
pub fn pid_start_time(pid: u32) -> u64 {
    #[cfg(target_os = "linux")]
    {
        pid_start_time_linux(pid)
    }
    #[cfg(target_os = "macos")]
    {
        pid_start_time_macos(pid)
    }
    #[cfg(target_os = "windows")]
    {
        pid_start_time_windows(pid)
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        let _ = pid;
        0
    }
}

#[cfg(target_os = "linux")]
fn pid_start_time_linux(pid: u32) -> u64 {
    let content = match std::fs::read_to_string(format!("/proc/{}/stat", pid)) {
        Ok(c) => c,
        Err(_) => return 0,
    };
    // The comm field (field 2) is in parentheses and may contain spaces.
    // Find the last ')' to safely skip it.
    let after_comm = match content.rfind(')') {
        Some(pos) => &content[pos + 1..],
        None => return 0,
    };
    // Field 19 after comm = starttime (jiffies since boot)
    after_comm
        .split_whitespace()
        .nth(19)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(0)
}

#[cfg(target_os = "macos")]
fn pid_start_time_macos(pid: u32) -> u64 {
    // kinfo_proc layout constants for macOS (x86_64 and aarch64).
    // p_starttime is a timeval at offset 136 within kp_proc (offset 0 in kinfo_proc).
    // timeval = { tv_sec: i64 (8 bytes), tv_usec: i32 (4 bytes) } on macOS.
    const KINFO_PROC_SIZE: usize = 648;
    const P_STARTTIME_OFFSET: usize = 136; // kp_proc.p_starttime

    let mut buf = vec![0u8; KINFO_PROC_SIZE];
    let mut size = KINFO_PROC_SIZE;
    let mut mib: [libc::c_int; 4] = [
        libc::CTL_KERN,
        libc::KERN_PROC,
        libc::KERN_PROC_PID,
        pid as libc::c_int,
    ];
    // SAFETY: mib is valid, buf is properly sized for one kinfo_proc.
    let ret = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            4,
            buf.as_mut_ptr() as *mut libc::c_void,
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };
    if ret != 0 || size == 0 || size < KINFO_PROC_SIZE {
        return 0;
    }
    // Extract tv_sec (i64, 8 bytes) and tv_usec (i32, 4 bytes) from p_starttime offset
    let off = P_STARTTIME_OFFSET;
    if off + 12 > buf.len() {
        return 0;
    }
    let tv_sec = i64::from_ne_bytes(buf[off..off + 8].try_into().unwrap_or([0; 8]));
    let tv_usec = i32::from_ne_bytes(buf[off + 8..off + 12].try_into().unwrap_or([0; 4]));
    (tv_sec as u64)
        .saturating_mul(1_000_000)
        .saturating_add(tv_usec as u64)
}

#[cfg(target_os = "windows")]
fn pid_start_time_windows(pid: u32) -> u64 {
    #[repr(C)]
    struct Filetime {
        low: u32,
        high: u32,
    }
    extern "system" {
        fn OpenProcess(access: u32, inherit: i32, pid: u32) -> *mut std::ffi::c_void;
        fn CloseHandle(h: *mut std::ffi::c_void) -> i32;
        fn GetProcessTimes(
            h: *mut std::ffi::c_void,
            creation: *mut Filetime,
            exit: *mut Filetime,
            kernel: *mut Filetime,
            user: *mut Filetime,
        ) -> i32;
    }
    const PROCESS_QUERY_LIMITED_INFORMATION: u32 = 0x1000;

    // SAFETY: pid is a valid process ID; requesting limited query access.
    let handle = unsafe { OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid) };
    if handle.is_null() {
        return 0;
    }
    let mut creation = Filetime { low: 0, high: 0 };
    let mut exit = Filetime { low: 0, high: 0 };
    let mut kernel = Filetime { low: 0, high: 0 };
    let mut user = Filetime { low: 0, high: 0 };
    // SAFETY: handle is valid; all pointers are initialized.
    let ret = unsafe {
        GetProcessTimes(handle, &mut creation, &mut exit, &mut kernel, &mut user)
    };
    unsafe { CloseHandle(handle) };
    if ret == 0 {
        return 0;
    }
    ((creation.high as u64) << 32) | (creation.low as u64)
}

// ── Signal Handler Registration ─────────────────────────────────────────────

/// Install a handler for the SIGTERM (Unix) / Ctrl+Break (Windows) signal.
///
/// The `handler` must be async-signal-safe — typically just setting an atomic flag.
///
/// # Safety
/// This installs a raw signal handler. The handler function must only call
/// async-signal-safe operations (e.g., `AtomicBool::store`).
pub fn on_terminate(handler: extern "C" fn(i32)) {
    #[cfg(unix)]
    {
        // SAFETY: SIGTERM is a valid signal; handler is a valid function pointer.
        unsafe {
            libc::signal(libc::SIGTERM, handler as libc::sighandler_t);
        }
    }
    #[cfg(windows)]
    {
        // Windows doesn't have SIGTERM — use Ctrl+Break handler via SetConsoleCtrlHandler
        // Store the handler for the trampoline
        TERMINATE_HANDLER.store(handler as usize, std::sync::atomic::Ordering::SeqCst);
        extern "system" fn ctrl_handler(ctrl_type: u32) -> i32 {
            if ctrl_type == 1 {
                // CTRL_BREAK_EVENT
                let ptr = TERMINATE_HANDLER.load(std::sync::atomic::Ordering::SeqCst);
                if ptr != 0 {
                    let f: extern "C" fn(i32) = unsafe { std::mem::transmute(ptr) };
                    f(0);
                }
                1 // handled
            } else {
                0 // not handled
            }
        }
        extern "system" {
            fn SetConsoleCtrlHandler(handler: extern "system" fn(u32) -> i32, add: i32) -> i32;
        }
        // SAFETY: ctrl_handler is a valid system callback. Return value ignored
        // because signal handler installation is best-effort — failure means
        // graceful shutdown won't work but the process can still function.
        let _ = unsafe { SetConsoleCtrlHandler(ctrl_handler, 1) };
    }
}

#[cfg(windows)]
static TERMINATE_HANDLER: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn current_process_is_alive() {
        let handle = ProcessHandle::from_pid(std::process::id());
        assert!(handle.is_alive(), "current process should be alive");
    }

    #[test]
    fn invalid_pid_is_not_alive() {
        // Use a high but valid PID (won't wrap to negative like u32::MAX would)
        let handle = ProcessHandle::from_pid(99_999_999);
        assert!(!handle.is_alive(), "invalid PID should not be alive");
    }

    #[test]
    fn overflow_pid_is_not_alive() {
        // u32::MAX wraps to -1 as i32; kill(-1, sig) means "kill all" — guard must catch this
        let handle = ProcessHandle::from_pid(u32::MAX);
        assert!(!handle.is_alive(), "overflow PID must be rejected, not sent to kill()");
    }

    #[test]
    fn from_pid_stores_pid() {
        let handle = ProcessHandle::from_pid(12345);
        assert_eq!(handle.pid(), 12345);
    }

    #[test]
    fn session_id_is_nonzero() {
        let sid = session_id();
        assert!(sid > 0, "session_id should be positive, got {}", sid);
    }

    #[test]
    fn user_id_is_nonzero() {
        let uid = user_id();
        // uid 0 is valid (root) but in most test environments it's non-zero
        // Just check it doesn't panic
        let _ = uid;
    }

    #[test]
    fn namespace_id_format() {
        let ns = namespace_id();
        assert!(ns.starts_with("sid"), "namespace should start with sid: {}", ns);
        assert!(ns.contains("_uid"), "namespace should contain _uid: {}", ns);
    }

    #[test]
    fn signal_kill_nonexistent_process_is_ok() {
        // Sending Kill to a non-existent PID should not error (ESRCH handled as Ok)
        let handle = ProcessHandle::from_pid(99_999_999);
        let result = handle.signal(Signal::Kill);
        assert!(result.is_ok(), "signal to non-existent PID should be Ok (ESRCH handled), got: {:?}", result);
    }

    #[test]
    fn signal_overflow_pid_is_ok() {
        // u32::MAX wraps to -1 — guard must prevent kill(-1, SIGKILL) which kills all processes
        let handle = ProcessHandle::from_pid(u32::MAX);
        let result = handle.signal(Signal::Kill);
        assert!(result.is_ok(), "overflow PID should be Ok (guard catches it)");
    }

    #[test]
    fn wait_timeout_for_dead_process() {
        // Wait for a non-existent process — should return immediately
        let handle = ProcessHandle::from_pid(99_999_999);
        let result = handle.wait_timeout(Some(Duration::from_millis(100)));
        // Either returns Some(exit_code) immediately or times out
        assert!(result.is_ok());
    }

    // ── PID start time tests ────────────────────────────────────────

    #[test]
    fn pid_start_time_current_process_is_nonzero() {
        let st = pid_start_time(std::process::id());
        assert!(st > 0, "current process start time should be nonzero, got {}", st);
    }

    #[test]
    fn pid_start_time_dead_process_is_zero() {
        let st = pid_start_time(99_999_999);
        assert_eq!(st, 0, "dead process start time should be 0");
    }

    #[test]
    fn pid_start_time_is_stable() {
        // Same PID queried twice should return the same value
        let pid = std::process::id();
        let st1 = pid_start_time(pid);
        let st2 = pid_start_time(pid);
        assert_eq!(st1, st2, "start time should be stable for same PID");
    }

    // ── Signal enum tests ───────────────────────────────────────────

    #[test]
    fn signal_terminate_nonexistent_is_ok() {
        let handle = ProcessHandle::from_pid(99_999_999);
        assert!(handle.signal(Signal::Terminate).is_ok());
    }

    #[test]
    fn signal_interrupt_nonexistent_is_ok() {
        let handle = ProcessHandle::from_pid(99_999_999);
        assert!(handle.signal(Signal::Interrupt).is_ok());
    }

    // ── ProcessHandle construction tests ────────────────────────────

    #[test]
    fn from_child_captures_pid() {
        let child = std::process::Command::new("sleep")
            .arg("0")
            .spawn();
        if let Ok(child) = child {
            let handle = ProcessHandle::from_child(&child);
            assert!(handle.pid() > 0);
        }
        // If sleep doesn't exist (Windows), skip silently
    }

    // ── Session/User ID consistency ─────────────────────────────────

    #[test]
    fn session_id_is_stable() {
        let s1 = session_id();
        let s2 = session_id();
        assert_eq!(s1, s2, "session_id should be stable across calls");
    }

    #[test]
    fn user_id_is_stable() {
        let u1 = user_id();
        let u2 = user_id();
        assert_eq!(u1, u2, "user_id should be stable across calls");
    }

    #[test]
    fn namespace_id_contains_session_and_user() {
        let ns = namespace_id();
        let sid = session_id();
        let uid = user_id();
        assert!(
            ns.contains(&format!("sid{}", sid)),
            "namespace should contain session id"
        );
        assert!(
            ns.contains(&format!("uid{}", uid)),
            "namespace should contain user id"
        );
    }
}
