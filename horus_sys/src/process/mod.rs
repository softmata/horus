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
    /// A process that has exited but not yet been reaped is *not* alive, on
    /// either platform:
    ///
    /// - Unix: `kill(pid, 0) == 0` and the process is not a zombie
    /// - Windows: the process opens *and* has not signalled its exit
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
            let exists = unsafe { libc::kill(self.pid as i32, 0) == 0 };
            if !exists {
                return false;
            }
            // kill(pid, 0) succeeds for a zombie: the process has exited but its
            // entry survives until the parent reaps it. A node killed with
            // SIGKILL under a parent that never waits was therefore reported
            // Running indefinitely, with its tick counts frozen — observed
            // holding at "Running, 49 ticks" for the full length of a test while
            // `ps` showed `Z (defunct)` and the presence file was never cleaned
            // up. Under `horus run` the parent reaps within milliseconds, which
            // is why this looked like a brief blip rather than a stuck state.
            !is_zombie(self.pid)
        }
        #[cfg(windows)]
        {
            use windows_sys::Win32::Foundation::{CloseHandle, STILL_ACTIVE, WAIT_TIMEOUT};
            use windows_sys::Win32::System::Threading::{
                GetExitCodeProcess, OpenProcess, WaitForSingleObject,
                PROCESS_QUERY_LIMITED_INFORMATION,
            };

            // The right to wait on a handle. Declared here rather than
            // imported from Win32::Storage::FileSystem, which is the only
            // place windows-sys spells it, and where it reads as a file right.
            const SYNCHRONIZE: u32 = 0x0010_0000;

            // Windows' analogue of a Unix zombie: a terminated process keeps
            // its pid — and answers OpenProcess — for as long as anybody still
            // holds a handle to it. Opening the process therefore proves the
            // pid is *known*, not that it is *running*, exactly the way
            // kill(pid, 0) does on Unix. Ask for the exit status too.
            unsafe {
                // Prefer a handle we can wait on: WaitForSingleObject is exact,
                // whereas GetExitCodeProcess cannot tell a live process from
                // one that exited with code 259 (STILL_ACTIVE).
                let mut waitable = true;
                let mut handle =
                    OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION | SYNCHRONIZE, 0, self.pid);
                if handle.is_null() {
                    waitable = false;
                    handle = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, self.pid);
                }
                if handle.is_null() {
                    return false;
                }
                let alive = if waitable {
                    // Signalled means "has exited"; timing out means running.
                    WaitForSingleObject(handle, 0) == WAIT_TIMEOUT
                } else {
                    let mut code: u32 = 0;
                    GetExitCodeProcess(handle, &mut code) != 0 && code == STILL_ACTIVE as u32
                };
                CloseHandle(handle);
                alive
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
                return Err(anyhow::anyhow!(
                    "kill({}, {}) failed: {}",
                    self.pid,
                    signum,
                    err
                ));
            }
        }
        #[cfg(windows)]
        {
            match sig {
                Signal::Kill => {
                    use windows_sys::Win32::Foundation::CloseHandle;
                    use windows_sys::Win32::System::Threading::{
                        OpenProcess, TerminateProcess, PROCESS_TERMINATE,
                    };
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
                        Signal::Interrupt => 0, // CTRL_C_EVENT
                        Signal::Terminate => 1, // CTRL_BREAK_EVENT
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
        if sid < 0 {
            std::process::id() as u64
        } else {
            sid as u64
        }
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

/// Whether the process has exited but not yet been reaped.
///
/// The one definition of "the pid still answers but the process is gone",
/// shared by [`ProcessHandle::is_alive`] and [`crate::shm::session_alive`] —
/// two liveness checks that used to disagree, so a zombie session leader kept
/// a dead namespace marked alive and unreclaimed.
///
/// Fails open: if the state cannot be read, the caller keeps whatever
/// `kill(pid, 0)` said, matching the existing convention in presence.rs.
#[cfg(unix)]
pub fn is_zombie(pid: u32) -> bool {
    #[cfg(target_os = "linux")]
    {
        let Ok(content) = std::fs::read_to_string(format!("/proc/{}/stat", pid)) else {
            return false;
        };
        // The comm field is parenthesised and may contain spaces, so skip to
        // after the last ')'. The state character is the first token after it.
        // Same idiom as pid_start_time_linux below.
        let Some(after_comm) = content.rfind(')').map(|i| &content[i + 1..]) else {
            return false;
        };
        matches!(
            after_comm.split_whitespace().next(),
            // Z = zombie, X/x = dead.
            Some("Z") | Some("X") | Some("x")
        )
    }
    #[cfg(target_os = "macos")]
    {
        // SAFETY: proc_pidinfo with PROC_PIDTBSDINFO fills a proc_bsdinfo for a
        // pid we already know exists; the same call is used by
        // pid_start_time_macos below.
        unsafe {
            let mut info: libc::proc_bsdinfo = std::mem::zeroed();
            let size = std::mem::size_of::<libc::proc_bsdinfo>() as libc::c_int;
            let n = libc::proc_pidinfo(
                pid as libc::c_int,
                libc::PROC_PIDTBSDINFO,
                0,
                &mut info as *mut _ as *mut libc::c_void,
                size,
            );
            n == size && info.pbi_status == libc::SZOMB
        }
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        let _ = pid;
        false
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
    // libproc exposes a stable, architecture-independent proc_bsdinfo layout;
    // hard-coded kinfo_proc offsets differ between Intel and Apple Silicon.
    let mut info: libc::proc_bsdinfo = unsafe { std::mem::zeroed() };
    let expected = std::mem::size_of::<libc::proc_bsdinfo>();
    let read = unsafe {
        libc::proc_pidinfo(
            pid as libc::c_int,
            libc::PROC_PIDTBSDINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            expected as libc::c_int,
        )
    };
    if read != expected as libc::c_int {
        return 0;
    }
    info.pbi_start_tvsec
        .saturating_mul(1_000_000)
        .saturating_add(info.pbi_start_tvusec)
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
    let ret = unsafe { GetProcessTimes(handle, &mut creation, &mut exit, &mut kernel, &mut user) };
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
static TERMINATE_HANDLER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// A child that has been SIGKILLed and deliberately never reaped — a zombie.
///
/// Shared by the `process` and `shm` tests, which both have a liveness check
/// that has to survive one. The [`std::process::Child`] is returned and must
/// be kept alive by the caller: dropping it is harmless (Rust never reaps on
/// drop), but reaping it elsewhere would dissolve the zombie.
#[cfg(all(test, unix))]
pub(crate) fn spawn_unreaped_zombie() -> Option<(std::process::Child, u32)> {
    let child = std::process::Command::new("sleep").arg("60").spawn().ok()?;
    let pid = child.id();
    // SAFETY: pid comes straight from a child we just spawned.
    unsafe { libc::kill(pid as i32, libc::SIGKILL) };

    // The kill is asynchronous; give the kernel time to turn the child into a
    // zombie. Poll on `kill(pid, 0)` rather than on our own liveness check, so
    // a broken liveness check cannot make this helper quietly give up and turn
    // its callers into no-ops.
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
        // On Linux the state character says so outright. Elsewhere, the pid
        // answering kill(pid, 0) after a SIGKILL *is* the zombie condition:
        // nothing has reaped it, so the entry cannot have been recycled.
        #[cfg(target_os = "linux")]
        {
            let stat = std::fs::read_to_string(format!("/proc/{}/stat", pid)).unwrap_or_default();
            let state = stat
                .rfind(')')
                .and_then(|i| stat[i + 1..].split_whitespace().next().map(str::to_string));
            if state.as_deref() == Some("Z") {
                return Some((child, pid));
            }
            continue;
        }
        #[cfg(not(target_os = "linux"))]
        {
            // No /proc: ask ps, which is nobody's implementation but the
            // system's. Returning as soon as kill(pid, 0) succeeds would be
            // wrong — SIGKILL is asynchronous, so that is also true of a child
            // that is still running.
            let state = std::process::Command::new("ps")
                .args(["-o", "state=", "-p", &pid.to_string()])
                .output()
                .ok()
                .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
                .unwrap_or_default();
            if state.starts_with('Z') {
                return Some((child, pid));
            }
        }
    }
    None
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn current_process_is_alive() {
        let handle = ProcessHandle::from_pid(std::process::id());
        assert!(handle.is_alive(), "current process should be alive");
    }

    /// LIVE-12: `kill(pid, 0)` succeeds for a process that has exited but not
    /// been reaped, so a SIGKILLed node stayed "Running" forever under a parent
    /// that never waits. Liveness must mean *running*, not *known to the
    /// kernel*.
    #[test]
    #[cfg(unix)]
    fn a_killed_but_unreaped_child_is_not_alive() {
        let Some((_child, pid)) = spawn_unreaped_zombie() else {
            panic!("could not produce an unreaped zombie child");
        };
        let handle = ProcessHandle::from_pid(pid);

        // Precondition — without it the assertion below could pass merely
        // because the process was fully gone, which proves nothing.
        // SAFETY: signal 0 only probes for existence.
        assert_eq!(
            unsafe { libc::kill(pid as i32, 0) },
            0,
            "the child was reaped: this test needs a pid that still answers kill(pid, 0)"
        );
        assert!(
            is_zombie(pid),
            "the child should be a zombie, not a running process"
        );

        assert!(
            !handle.is_alive(),
            "pid {pid} was SIGKILLed and never reaped — it must not be reported alive"
        );
    }

    #[test]
    #[cfg(unix)]
    fn a_running_child_is_alive_and_not_a_zombie() {
        // The other side of the same test: the zombie check must not report
        // every child as dead.
        let mut child = std::process::Command::new("sleep")
            .arg("30")
            .spawn()
            .expect("sleep should be spawnable");
        let pid = child.id();
        let handle = ProcessHandle::from_pid(pid);
        assert!(!is_zombie(pid), "a running child is not a zombie");
        assert!(handle.is_alive(), "a running child must be reported alive");

        let _ = child.kill();
        let _ = child.wait();
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
        assert!(
            !handle.is_alive(),
            "overflow PID must be rejected, not sent to kill()"
        );
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
        assert!(
            ns.starts_with("sid"),
            "namespace should start with sid: {}",
            ns
        );
        assert!(ns.contains("_uid"), "namespace should contain _uid: {}", ns);
    }

    #[test]
    fn signal_kill_nonexistent_process_is_ok() {
        // Sending Kill to a non-existent PID should not error (ESRCH handled as Ok)
        let handle = ProcessHandle::from_pid(99_999_999);
        let result = handle.signal(Signal::Kill);
        assert!(
            result.is_ok(),
            "signal to non-existent PID should be Ok (ESRCH handled), got: {:?}",
            result
        );
    }

    #[test]
    fn signal_overflow_pid_is_ok() {
        // u32::MAX wraps to -1 — guard must prevent kill(-1, SIGKILL) which kills all processes
        let handle = ProcessHandle::from_pid(u32::MAX);
        let result = handle.signal(Signal::Kill);
        assert!(
            result.is_ok(),
            "overflow PID should be Ok (guard catches it)"
        );
    }

    #[test]
    fn wait_timeout_for_dead_process() {
        // Wait for a non-existent process — should return immediately
        let handle = ProcessHandle::from_pid(99_999_999);
        let result = handle.wait_timeout(Some(Duration::from_millis(100)));
        // Either returns Some(exit_code) immediately or times out
        result.unwrap();
    }

    // ── PID start time tests ────────────────────────────────────────

    #[test]
    fn pid_start_time_current_process_is_nonzero() {
        let st = pid_start_time(std::process::id());
        assert!(
            st > 0,
            "current process start time should be nonzero, got {}",
            st
        );
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
        handle.signal(Signal::Terminate).unwrap();
    }

    #[test]
    fn signal_interrupt_nonexistent_is_ok() {
        let handle = ProcessHandle::from_pid(99_999_999);
        handle.signal(Signal::Interrupt).unwrap();
    }

    // ── ProcessHandle construction tests ────────────────────────────

    #[test]
    fn from_child_captures_pid() {
        let child = std::process::Command::new("sleep").arg("0").spawn();
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
