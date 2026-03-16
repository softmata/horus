//! macOS process discovery via sysctl and libproc.

use super::ProcessInfo;
use std::collections::HashMap;
use std::sync::{OnceLock, RwLock};
use std::time::Instant;

/// CPU usage cache: PID → (user_ns, system_ns, timestamp). Evicted when size exceeds 1024.
fn cpu_cache() -> &'static RwLock<HashMap<u32, (u64, u64, Instant)>> {
    static CACHE: OnceLock<RwLock<HashMap<u32, (u64, u64, Instant)>>> = OnceLock::new();
    CACHE.get_or_init(|| RwLock::new(HashMap::new()))
}

const CPU_CACHE_MAX_SIZE: usize = 1024;

pub fn get_process_info(pid: u32) -> anyhow::Result<ProcessInfo> {
    let cmdline = get_cmdline(pid).unwrap_or_default();
    let working_dir = get_cwd(pid).unwrap_or_default();
    let memory_kb = get_memory(pid).unwrap_or(0);
    let cpu_percent = calculate_cpu_usage(pid);
    let start_time = get_start_time(pid).unwrap_or_else(|| "Unknown".to_string());

    Ok(ProcessInfo {
        pid,
        cmdline,
        working_dir,
        cpu_percent,
        memory_kb,
        start_time,
    })
}

// kinfo_proc layout: the PID lives at offset 24 (kp_proc.p_pid) on both
// x86_64 and aarch64 macOS. We use a raw byte buffer + offset extraction
// to avoid depending on libc::kinfo_proc which is unavailable during
// cross-compilation from Linux.
const KINFO_PROC_SIZE: usize = 648; // sizeof(struct kinfo_proc) on macOS
const KP_PROC_PID_OFFSET: usize = 24; // offsetof(kinfo_proc, kp_proc.p_pid)

/// List all PIDs via sysctl(KERN_PROC_ALL).
pub fn list_pids() -> Vec<u32> {
    let mut mib: [i32; 4] = [
        libc::CTL_KERN,
        libc::KERN_PROC,
        libc::KERN_PROC_ALL,
        0,
    ];

    let mut size: usize = 0;

    // First call to get buffer size
    // SAFETY: mib is a valid 4-element array; null output buffer queries size.
    let ret = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            4,
            std::ptr::null_mut(),
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };

    if ret != 0 || size == 0 {
        return Vec::new();
    }

    let mut buf: Vec<u8> = vec![0u8; size];

    // SAFETY: buf is properly sized; sysctl fills it with kinfo_proc structs.
    let ret = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            4,
            buf.as_mut_ptr() as *mut _,
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };

    if ret != 0 {
        return Vec::new();
    }

    let count = size / KINFO_PROC_SIZE;
    let mut pids = Vec::with_capacity(count);
    for i in 0..count {
        let base = i * KINFO_PROC_SIZE + KP_PROC_PID_OFFSET;
        if base + 4 <= buf.len() {
            // SAFETY: reading an i32 from a properly aligned kinfo_proc struct
            let pid = i32::from_ne_bytes([buf[base], buf[base + 1], buf[base + 2], buf[base + 3]]);
            if pid > 0 {
                pids.push(pid as u32);
            }
        }
    }
    pids
}

fn get_cmdline(pid: u32) -> Option<String> {
    let mut mib: [i32; 3] = [
        1,  // CTL_KERN
        49, // KERN_PROCARGS2
        pid as i32,
    ];

    let mut size: usize = 0;

    // First call to get size
    // SAFETY: mib is a valid 3-element array for KERN_PROCARGS2; null output buffer queries size.
    let result = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            3,
            std::ptr::null_mut(),
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };

    if result != 0 || size == 0 {
        return None;
    }

    let mut buf = vec![0u8; size];

    // Second call to get data
    // SAFETY: buf is allocated to the size returned by the first sysctl call.
    let result = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            3,
            buf.as_mut_ptr() as *mut _,
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };

    if result != 0 {
        return None;
    }

    // Parse KERN_PROCARGS2 format: argc (4 bytes) + exec_path + NULLs + args
    if buf.len() < 4 {
        return None;
    }

    let argc_raw = i32::from_ne_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if argc_raw < 0 {
        return None;
    }
    let argc = argc_raw as usize;
    let mut pos = 4;

    // Skip executable path
    while pos < buf.len() && buf[pos] != 0 {
        pos += 1;
    }
    // Skip null terminators
    while pos < buf.len() && buf[pos] == 0 {
        pos += 1;
    }

    // Collect arguments
    let mut args = Vec::new();
    for _ in 0..argc {
        if pos >= buf.len() {
            break;
        }
        let start = pos;
        while pos < buf.len() && buf[pos] != 0 {
            pos += 1;
        }
        if start < pos {
            if let Ok(arg) = std::str::from_utf8(&buf[start..pos]) {
                args.push(arg.to_string());
            }
        }
        pos += 1;
    }

    if args.is_empty() {
        None
    } else {
        Some(args.join(" "))
    }
}

fn get_cwd(pid: u32) -> Option<String> {
    const PROC_PIDVNODEPATHINFO: i32 = 9;
    const MAXPATHLEN: usize = 1024;

    #[repr(C)]
    struct VnodePathInfo {
        pvi_cdir: VnodeInfoPath,
        _pvi_rdir: VnodeInfoPath,
    }

    #[repr(C)]
    struct VnodeInfoPath {
        _vip_vi: [u8; 152],
        vip_path: [u8; MAXPATHLEN],
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    // SAFETY: VnodePathInfo is repr(C) where all-zeros is valid.
    let mut info: VnodePathInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<VnodePathInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized.
    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDVNODEPATHINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return None;
    }

    let path_bytes = &info.pvi_cdir.vip_path;
    let end = path_bytes.iter().position(|&b| b == 0).unwrap_or(path_bytes.len());
    std::str::from_utf8(&path_bytes[..end]).ok().map(|s| s.to_string())
}

fn get_memory(pid: u32) -> Option<u64> {
    const PROC_PIDTASKINFO: i32 = 4;

    #[repr(C)]
    struct TaskInfo {
        _pti_virtual_size: u64,
        pti_resident_size: u64,
        pti_total_user: u64,
        pti_total_system: u64,
        _pti_threads_user: u64,
        _pti_threads_system: u64,
        _pti_policy: i32,
        _pti_faults: i32,
        _pti_pageins: i32,
        _pti_cow_faults: i32,
        _pti_messages_sent: i32,
        _pti_messages_received: i32,
        _pti_syscalls_mach: i32,
        _pti_syscalls_unix: i32,
        _pti_csw: i32,
        _pti_threadnum: i32,
        _pti_numrunning: i32,
        _pti_priority: i32,
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    // SAFETY: TaskInfo is repr(C) where all-zeros is valid.
    let mut info: TaskInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<TaskInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized.
    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDTASKINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return None;
    }

    Some(info.pti_resident_size / 1024)
}

fn calculate_cpu_usage(pid: u32) -> f32 {
    const PROC_PIDTASKINFO: i32 = 4;

    #[repr(C)]
    struct TaskInfo {
        _pti_virtual_size: u64,
        _pti_resident_size: u64,
        pti_total_user: u64,
        pti_total_system: u64,
        _padding: [u8; 64],
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    // SAFETY: TaskInfo is repr(C) where all-zeros is valid.
    let mut info: TaskInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<TaskInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized.
    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDTASKINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return 0.0;
    }

    let total_time = info.pti_total_user + info.pti_total_system;
    let num_cpus = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1);

    if let Ok(mut cache) = cpu_cache().write() {
        let now = Instant::now();

        if let Some((prev_user, prev_system, prev_time)) = cache.get(&pid) {
            let time_delta = now.duration_since(*prev_time).as_secs_f32();
            if time_delta > 0.0 {
                let prev_total = prev_user + prev_system;
                let cpu_delta_ns = total_time.saturating_sub(prev_total) as f32;
                let cpu_percent = (cpu_delta_ns / 1_000_000_000.0 / time_delta) * 100.0;
                cache.insert(pid, (info.pti_total_user, info.pti_total_system, now));
                return cpu_percent.min(100.0 * num_cpus as f32);
            }
        }

        if cache.len() >= CPU_CACHE_MAX_SIZE {
            let cutoff = Instant::now() - std::time::Duration::from_secs(60);
            cache.retain(|_, (_, _, ts)| *ts > cutoff);
        }
        cache.insert(pid, (info.pti_total_user, info.pti_total_system, now));
    }

    0.0
}

fn get_start_time(pid: u32) -> Option<String> {
    const PROC_PIDTBSDINFO: i32 = 3;

    #[repr(C)]
    struct BsdInfo {
        _pbi_flags: u32,
        _pbi_status: u32,
        _pbi_xstatus: u32,
        _pbi_pid: u32,
        _pbi_ppid: u32,
        _pbi_uid: u32,
        _pbi_gid: u32,
        _pbi_ruid: u32,
        _pbi_rgid: u32,
        _pbi_svuid: u32,
        _pbi_svgid: u32,
        _rfu_1: u32,
        _pbi_comm: [u8; 16],
        _pbi_name: [u8; 32],
        _pbi_nfiles: u32,
        _pbi_pgid: u32,
        _pbi_pjobc: u32,
        _e_tdev: u32,
        _e_tpgid: u32,
        _pbi_nice: i32,
        pbi_start_tvsec: u64,
        _pbi_start_tvusec: u64,
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    // SAFETY: BsdInfo is repr(C) where all-zeros is valid.
    let mut info: BsdInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<BsdInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized.
    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDTBSDINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return None;
    }

    let start = std::time::UNIX_EPOCH + std::time::Duration::from_secs(info.pbi_start_tvsec);
    if let Ok(elapsed) = std::time::SystemTime::now().duration_since(start) {
        Some(super::format_duration(elapsed))
    } else {
        None
    }
}
