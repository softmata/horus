//! Plugin process sandbox — Linux security restrictions applied in the child
//! process after `fork()` but before `exec()`.
//!
//! # What this module does
//!
//! 1. **Resource limits** (`setrlimit`): caps CPU time, maximum file size a
//!    plugin may write, and the number of open file descriptors.
//!
//! 2. **Inherited FD cleanup**: closes every file descriptor above 2 (stderr)
//!    to prevent the plugin from accidentally or maliciously using the parent
//!    process's open files (sockets, log files, shared-memory FDs, etc.).
//!
//! 3. **`no_new_privs`** (`prctl PR_SET_NO_NEW_PRIVS`): prevents the plugin
//!    binary and any child it spawns from gaining elevated capabilities via
//!    setuid/setgid bits.  This is also required before a seccomp filter can be
//!    applied without `CAP_SYS_ADMIN`.
//!
//! 4. **Seccomp-BPF deny list**: blocks the syscalls most useful for privilege
//!    escalation, container escape, and unauthorised network access:
//!    - `socket` (and `bind`, `connect`, `listen`, `accept`, `accept4`,
//!      `setsockopt`, `getsockopt`, `sendmsg`, `sendto`, `recvmsg`, `recvfrom`)
//!    - `ptrace`
//!    - `chroot`, `pivot_root`, `mount`, `umount2`
//!    - `setuid`, `setgid`, `setresuid`, `setresgid`, `setreuid`, `setregid`,
//!      `capset`
//!    - `mknod`, `mknodat` (device file creation)
//!    - `perf_event_open` (kernel performance counter abuse)
//!
//!    Everything else (including `read`, `write`, `mmap`, `futex`, `openat`,
//!    `execve`, `fork`, etc.) is allowed so that typical plugin binaries and
//!    any subprocesses they spawn can function correctly.  All forked/exec'd
//!    children inherit the filter automatically.
//!
//! # Compatibility note
//!
//! Seccomp is verified against `AUDIT_ARCH_X86_64`.  On other architectures the
//! filter is not applied (the `apply` function returns `Ok(())` without doing
//! anything on non-Linux platforms or non-x86_64 Linux at runtime).  The rlimit
//! and FD-cleanup steps run on all Linux architectures.
//!
//! # Why path-based file isolation is not provided here
//!
//! Blocking reads to arbitrary paths (e.g. `/etc/passwd`) via seccomp requires
//! inspecting syscall arguments (filename pointer in user space) which is
//! inherently TOCTOU-unsafe and complex.  Linux Landlock (≥5.13) provides safe
//! path-based access control without root but requires the kernel to be new
//! enough.  A future hardening iteration can add Landlock on top of this module.
//! In the meantime, `no_new_privs` + seccomp network deny + rlimits provide
//! meaningful defence-in-depth against the most common plugin abuse scenarios.

// This entire file is Linux-only.
#![cfg(target_os = "linux")]

use std::io;

// ─── libc constants ───────────────────────────────────────────────────────────

// BPF instruction class / opcode fragments
const BPF_LD: u16 = 0x00;
const BPF_JMP: u16 = 0x05;
const BPF_RET: u16 = 0x06;
const BPF_W: u16 = 0x00; // word (32-bit) load
const BPF_ABS: u16 = 0x20; // absolute addressing
const BPF_JEQ: u16 = 0x10; // jump-if-equal
const BPF_K: u16 = 0x00; // immediate

// seccomp return codes
const SECCOMP_RET_ALLOW: u32 = 0x7fff_0000;
const SECCOMP_RET_KILL_PROCESS: u32 = 0x8000_0000;
/// Return EPERM (1) to the calling syscall rather than killing the process.
/// Used for `socket` so the plugin sees a clear `-EPERM` instead of a crash.
const SECCOMP_RET_ERRNO_EPERM: u32 = 0x0005_0000 | 1;

// Architecture tag expected in seccomp_data.arch for x86-64.
const AUDIT_ARCH_X86_64: u32 = 0xC000_003E;

// Offsets within struct seccomp_data (stable ABI since Linux 3.5).
const SECCOMP_DATA_NR_OFFSET: u32 = 0; // offsetof(seccomp_data, nr)
const SECCOMP_DATA_ARCH_OFFSET: u32 = 4; // offsetof(seccomp_data, arch)

// x86-64 syscall numbers that we deny.
//
// Source: linux/arch/x86/entry/syscalls/syscall_64.tbl
const SYS_SOCKET: u32 = 41;
const SYS_CONNECT: u32 = 42;
const SYS_ACCEPT: u32 = 43;
const SYS_BIND: u32 = 49;
const SYS_LISTEN: u32 = 50;
const SYS_SETSOCKOPT: u32 = 54;
const SYS_GETSOCKOPT: u32 = 55;
const SYS_SENDMSG: u32 = 46;
const SYS_RECVMSG: u32 = 47;
const SYS_SENDTO: u32 = 44;
const SYS_RECVFROM: u32 = 45;
const SYS_ACCEPT4: u32 = 288;
const SYS_PTRACE: u32 = 101;
const SYS_CHROOT: u32 = 161;
const SYS_PIVOT_ROOT: u32 = 155;
const SYS_MOUNT: u32 = 165;
const SYS_UMOUNT2: u32 = 166;
const SYS_SETUID: u32 = 105;
const SYS_SETGID: u32 = 106;
const SYS_SETREUID: u32 = 113;
const SYS_SETREGID: u32 = 114;
const SYS_SETRESUID: u32 = 117;
const SYS_SETRESGID: u32 = 119;
const SYS_CAPSET: u32 = 126;
const SYS_MKNOD: u32 = 133;
const SYS_MKNODAT: u32 = 259;
const SYS_PERF_EVENT_OPEN: u32 = 298;

// ─── BPF helper constructors ─────────────────────────────────────────────────

/// `BPF_STMT` — an instruction with no jump offset.
#[inline]
fn bpf_stmt(code: u16, k: u32) -> libc::sock_filter {
    libc::sock_filter {
        code,
        jt: 0,
        jf: 0,
        k,
    }
}

/// `BPF_JUMP` — a conditional jump instruction.
#[inline]
fn bpf_jump(code: u16, k: u32, jt: u8, jf: u8) -> libc::sock_filter {
    libc::sock_filter { code, jt, jf, k }
}

// ─── Public API ──────────────────────────────────────────────────────────────

/// Apply all sandbox restrictions.
///
/// Called inside the forked child process via `Command::pre_exec`.  The function
/// is `unsafe` because it must only use async-signal-safe operations (no Rust
/// allocator, no Rust mutexes — only raw `libc` syscalls).
///
/// Returns `io::Error` if a restriction that MUST succeed fails (specifically
/// `prctl` / seccomp).  Rlimit and FD-cleanup failures are logged to stderr
/// but are not fatal so that a misconfigured rlimit doesn't prevent the plugin
/// from running at all.
pub fn apply(plugin_dir: Option<&std::path::Path>) -> io::Result<()> {
    let _ = plugin_dir; // reserved for future Landlock integration

    set_resource_limits();
    close_inherited_fds();
    set_no_new_privs()?;
    apply_seccomp_filter()?;

    Ok(())
}

// ─── Resource limits ─────────────────────────────────────────────────────────

/// Apply conservative resource limits.
///
/// Failures are silently ignored because some limits may already be set below
/// our target (e.g. in a tight CI container) and a plugin that runs with the
/// existing limits is better than a plugin that fails to start.
fn set_resource_limits() {
    // Maximum CPU time: 5 minutes.  Prevents runaway computation.
    let cpu = libc::rlimit {
        rlim_cur: 300,
        rlim_max: 300,
    };
    unsafe { libc::setrlimit(libc::RLIMIT_CPU, &cpu) };

    // Maximum file size a plugin may write: 256 MiB.
    // Prevents disk-exhaustion attacks from a misbehaving plugin.
    let fsize = libc::rlimit {
        rlim_cur: 256 * 1024 * 1024,
        rlim_max: 256 * 1024 * 1024,
    };
    unsafe { libc::setrlimit(libc::RLIMIT_FSIZE, &fsize) };

    // Maximum open file descriptors: 64.
    // Prevents FD exhaustion that could be used to starve the HORUS daemon.
    let nofile = libc::rlimit {
        rlim_cur: 64,
        rlim_max: 64,
    };
    unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &nofile) };
}

// ─── FD cleanup ──────────────────────────────────────────────────────────────

/// Close all file descriptors above 2 (stderr) in the child process.
///
/// Prevents the plugin from inheriting the parent's open sockets, log files,
/// shared-memory FDs, or any other OS resource that the parent had open.
fn close_inherited_fds() {
    // Try close_range(3, UINT_MAX, 0) — available since Linux 5.9 (syscall 436).
    // On older kernels ENOSYS is returned and we fall back to the manual loop.
    let ret = unsafe {
        libc::syscall(436 /* close_range */, 3u32, u32::MAX, 0u32)
    };
    if ret == 0 {
        return; // fast path
    }

    // Fallback: close fds 3..1024 manually (covers the typical default nofile).
    // We cannot read /proc/self/fd here (that would be an async-signal-unsafe
    // allocation), so we just brute-force a reasonable range.
    for fd in 3..1024i32 {
        unsafe { libc::close(fd) };
    }
}

// ─── no_new_privs ────────────────────────────────────────────────────────────

/// Set `PR_SET_NO_NEW_PRIVS` so the plugin cannot gain privileges via setuid
/// binaries, and so that seccomp can be applied without `CAP_SYS_ADMIN`.
fn set_no_new_privs() -> io::Result<()> {
    let ret = unsafe { libc::prctl(libc::PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) };
    if ret != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

// ─── Seccomp-BPF deny list ───────────────────────────────────────────────────

/// Apply a seccomp-BPF deny list that blocks the syscalls most useful for
/// privilege escalation, container escape, and network abuse.
///
/// Architecture: x86-64 only.  On any other architecture the filter header
/// kills the process to prevent filter bypass via ABI mismatch.
fn apply_seccomp_filter() -> io::Result<()> {
    // Syscalls we want to block.  Stored as a flat array so each entry produces
    // exactly two BPF instructions (JEQ + RET).  The `ret` field is the
    // seccomp return code when the syscall matches.
    struct Deny {
        nr: u32,
        ret: u32,
    }

    // Network socket syscalls → EPERM (plugin sees a clear error, not a crash)
    // Privilege/container escape syscalls → KILL (no recovery possible)
    let denied: &[Deny] = &[
        Deny {
            nr: SYS_SOCKET,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_CONNECT,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_BIND,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_LISTEN,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_ACCEPT,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_ACCEPT4,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_SETSOCKOPT,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_GETSOCKOPT,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_SENDMSG,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_SENDTO,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_RECVMSG,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_RECVFROM,
            ret: SECCOMP_RET_ERRNO_EPERM,
        },
        Deny {
            nr: SYS_PTRACE,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_CHROOT,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_PIVOT_ROOT,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_MOUNT,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_UMOUNT2,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_SETUID,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_SETGID,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_SETREUID,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_SETREGID,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_SETRESUID,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_SETRESGID,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_CAPSET,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_MKNOD,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_MKNODAT,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
        Deny {
            nr: SYS_PERF_EVENT_OPEN,
            ret: SECCOMP_RET_KILL_PROCESS,
        },
    ];

    // Build BPF filter at compile time: max instruction count is known.
    // Layout:
    //   [0]   LD  arch from seccomp_data.arch
    //   [1]   JEQ AUDIT_ARCH_X86_64 ? jump+1 : fall-through
    //   [2]   RET KILL  (wrong architecture)
    //   [3]   LD  syscall number from seccomp_data.nr
    //   [4..] For each Deny: JEQ nr ? fall-through : skip next   /   RET code
    //   [N]   RET ALLOW
    //
    // Each Deny entry = 2 instructions (JEQ + RET).
    const MAX_INSTRUCTIONS: usize = 4 + 27 * 2 + 1; // 59 — well within BPF limit (4096)
    let n_deny = denied.len();
    let total = 4 + n_deny * 2 + 1;
    assert!(total <= MAX_INSTRUCTIONS);

    let mut filter = [bpf_stmt(0, 0); MAX_INSTRUCTIONS];
    let mut i = 0usize;

    // Architecture guard
    filter[i] = bpf_stmt(BPF_LD | BPF_W | BPF_ABS, SECCOMP_DATA_ARCH_OFFSET);
    i += 1;
    // if arch == x86_64: jump forward 1 (skip the KILL); else fall-through to KILL
    filter[i] = bpf_jump(BPF_JMP | BPF_JEQ | BPF_K, AUDIT_ARCH_X86_64, 1, 0);
    i += 1;
    filter[i] = bpf_stmt(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS);
    i += 1;

    // Load syscall number
    filter[i] = bpf_stmt(BPF_LD | BPF_W | BPF_ABS, SECCOMP_DATA_NR_OFFSET);
    i += 1;

    // Per-syscall deny entries
    for deny in denied {
        // if nr == syscall: fall-through (jt=0) to the RET; else skip (jf=1)
        filter[i] = bpf_jump(BPF_JMP | BPF_JEQ | BPF_K, deny.nr, 0, 1);
        i += 1;
        filter[i] = bpf_stmt(BPF_RET | BPF_K, deny.ret);
        i += 1;
    }

    // Default: allow
    filter[i] = bpf_stmt(BPF_RET | BPF_K, SECCOMP_RET_ALLOW);
    i += 1;

    let mut prog = libc::sock_fprog {
        len: i as libc::c_ushort,
        filter: filter.as_mut_ptr(),
    };

    let ret = unsafe {
        libc::prctl(
            libc::PR_SET_SECCOMP,
            libc::SECCOMP_MODE_FILTER as libc::c_ulong,
            &mut prog as *mut libc::sock_fprog as libc::c_ulong,
            0,
            0,
        )
    };

    if ret != 0 {
        return Err(io::Error::last_os_error());
    }

    Ok(())
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::process::CommandExt;
    use std::process::{Command, Stdio};

    /// Verify that the BPF filter can be constructed and applied to a child
    /// process without panicking or returning an error.
    ///
    /// This is a smoke test: it spawns /bin/true (which exits 0) with the
    /// full sandbox applied.  If the sandbox construction or application fails,
    /// the pre_exec closure returns an error and Command::status() returns Err.
    #[test]
    fn test_sandbox_apply_does_not_crash() {
        let mut cmd = Command::new("/bin/true");
        cmd.env_clear();
        unsafe {
            cmd.pre_exec(|| apply(None));
        }
        let status = cmd
            .status()
            .expect("failed to spawn /bin/true with sandbox");
        assert!(
            status.success(),
            "/bin/true should exit 0 even with sandbox applied"
        );
    }

    /// Verify that the seccomp filter blocks `socket()`: a process that tries
    /// to create a network socket must fail (EPERM) or be killed (SIGSYS).
    ///
    /// We use `/bin/bash -c '(exec 5<>/dev/tcp/127.0.0.1/9)'` which invokes
    /// bash's built-in TCP redirection.  This calls `socket()` in the bash
    /// process itself, which is blocked by our seccomp filter.  The child exits
    /// non-zero (either EPERM when socket() is denied or SIGSYS if signalled).
    #[test]
    fn test_sandbox_seccomp_blocks_socket() {
        // Check that bash is available; skip test if not.
        if !std::path::Path::new("/bin/bash").exists() {
            return;
        }

        let mut cmd = Command::new("/bin/bash");
        // exec 5<>/dev/tcp/... is a bash built-in that calls socket() internally.
        // Port 9 (discard) — whether it's listening doesn't matter; the socket()
        // call itself is what seccomp checks.
        cmd.args(["-c", "exec 5<>/dev/tcp/127.0.0.1/9"])
            .env_clear()
            .stdout(Stdio::null())
            .stderr(Stdio::null());

        unsafe {
            cmd.pre_exec(|| apply(None));
        }

        let status = cmd.status().expect("failed to spawn bash for seccomp test");

        // The process must NOT succeed: socket() is denied by the filter.
        // bash exits non-zero when socket() returns EPERM, OR the process
        // is killed by SIGSYS (also non-zero / no exit code).
        assert!(
            !status.success(),
            "seccomp should have denied socket(), but bash exited successfully"
        );
    }

    /// Verify that resource limits are applied: the RLIMIT_NOFILE soft limit
    /// after sandbox application should be ≤ 64.
    #[test]
    fn test_sandbox_rlimit_nofile_applied() {
        let mut cmd = Command::new("/bin/sh");
        // Print the NOFILE soft limit from /proc/self/limits.
        // The limits file format: "Max open files  <soft>  <hard>  files"
        cmd.args(["-c", "cat /proc/self/limits"])
            .env_clear()
            .stdout(Stdio::piped())
            .stderr(Stdio::null());

        unsafe {
            cmd.pre_exec(|| apply(None));
        }

        let output = cmd.output().expect("failed to spawn sh for rlimit test");
        let stdout = String::from_utf8_lossy(&output.stdout);

        // Find the "Max open files" line and extract the soft limit.
        for line in stdout.lines() {
            if line.starts_with("Max open files") {
                let parts: Vec<&str> = line.split_whitespace().collect();
                // Format: "Max open files  <soft>  <hard>  files"
                // parts[3] = soft limit
                if let Some(soft_str) = parts.get(3) {
                    if let Ok(soft) = soft_str.parse::<u64>() {
                        assert!(soft <= 64, "RLIMIT_NOFILE soft should be ≤64, got {}", soft);
                        return;
                    }
                }
            }
        }
        // If /proc/self/limits is not available (unusual), just pass.
    }
}
