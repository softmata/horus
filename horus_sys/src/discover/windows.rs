//! Windows process discovery via Win32 API.

use super::ProcessInfo;
use std::collections::HashMap;
use std::sync::{OnceLock, RwLock};
use std::time::Instant;

/// CPU usage cache: PID → (total_100ns, timestamp). Evicted when size exceeds 1024.
fn cpu_cache() -> &'static RwLock<HashMap<u32, (u64, Instant)>> {
    static CACHE: OnceLock<RwLock<HashMap<u32, (u64, Instant)>>> = OnceLock::new();
    CACHE.get_or_init(|| RwLock::new(HashMap::new()))
}

const CPU_CACHE_MAX_SIZE: usize = 1024;

// Win32 constants
const PROCESS_QUERY_INFORMATION: u32 = 0x0400;
const PROCESS_VM_READ: u32 = 0x0010;
const PROCESS_QUERY_LIMITED_INFORMATION: u32 = 0x1000;
const TH32CS_SNAPPROCESS: u32 = 0x00000002;
const MAX_PATH: usize = 260;

extern "system" {
    fn OpenProcess(dwDesiredAccess: u32, bInheritHandle: i32, dwProcessId: u32) -> *mut std::ffi::c_void;
    fn CloseHandle(hObject: *mut std::ffi::c_void) -> i32;
    fn QueryFullProcessImageNameW(hProcess: *mut std::ffi::c_void, dwFlags: u32, lpExeName: *mut u16, lpdwSize: *mut u32) -> i32;
    fn CreateToolhelp32Snapshot(dwFlags: u32, th32ProcessID: u32) -> *mut std::ffi::c_void;
    fn Process32First(hSnapshot: *mut std::ffi::c_void, lppe: *mut ProcessEntry32) -> i32;
    fn Process32Next(hSnapshot: *mut std::ffi::c_void, lppe: *mut ProcessEntry32) -> i32;
}

#[repr(C)]
struct ProcessEntry32 {
    dw_size: u32,
    cnt_usage: u32,
    th32_process_id: u32,
    th32_default_heap_id: usize,
    th32_module_id: u32,
    cnt_threads: u32,
    th32_parent_process_id: u32,
    pc_pri_class_base: i32,
    dw_flags: u32,
    sz_exe_file: [u16; MAX_PATH],
}

pub fn get_process_info(pid: u32) -> anyhow::Result<ProcessInfo> {
    // SAFETY: pid is a valid process ID; requesting query-only access.
    let handle = unsafe { OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, 0, pid) };

    if handle.is_null() {
        return Ok(ProcessInfo { pid, ..Default::default() });
    }

    let cmdline = get_cmdline(pid).unwrap_or_default();
    let memory_kb = get_memory(handle).unwrap_or(0);
    let cpu_percent = calculate_cpu_usage(pid, handle);
    let start_time = get_start_time(handle).unwrap_or_else(|| "Unknown".to_string());

    // SAFETY: handle is valid from OpenProcess above.
    unsafe { CloseHandle(handle) };

    Ok(ProcessInfo {
        pid,
        cmdline,
        working_dir: String::new(), // Windows doesn't easily expose cwd
        cpu_percent,
        memory_kb,
        start_time,
    })
}

/// List all PIDs via CreateToolhelp32Snapshot.
pub fn list_pids() -> Vec<u32> {
    // SAFETY: TH32CS_SNAPPROCESS takes a snapshot of all processes.
    let snapshot = unsafe { CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0) };
    if snapshot.is_null() {
        return Vec::new();
    }

    let mut entry = ProcessEntry32 {
        dw_size: std::mem::size_of::<ProcessEntry32>() as u32,
        cnt_usage: 0,
        th32_process_id: 0,
        th32_default_heap_id: 0,
        th32_module_id: 0,
        cnt_threads: 0,
        th32_parent_process_id: 0,
        pc_pri_class_base: 0,
        dw_flags: 0,
        sz_exe_file: [0u16; MAX_PATH],
    };

    let mut pids = Vec::new();

    // SAFETY: snapshot is valid; entry is properly sized with dw_size set.
    if unsafe { Process32First(snapshot, &mut entry) } != 0 {
        pids.push(entry.th32_process_id);
        // SAFETY: snapshot and entry are valid from above.
        while unsafe { Process32Next(snapshot, &mut entry) } != 0 {
            pids.push(entry.th32_process_id);
        }
    }

    // SAFETY: snapshot is a valid handle.
    unsafe { CloseHandle(snapshot as *mut _) };

    pids
}

fn get_cmdline(pid: u32) -> Option<String> {
    use std::ffi::OsString;
    use std::os::windows::ffi::OsStringExt;

    const MAX_CMD_PATH: usize = 32768;

    // SAFETY: pid is a valid process ID; requesting limited query access.
    let handle = unsafe { OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid) };
    if handle.is_null() {
        return None;
    }

    let mut buf = vec![0u16; MAX_CMD_PATH];
    let mut size = MAX_CMD_PATH as u32;

    // SAFETY: handle is valid; buf is properly sized; size is in/out parameter.
    let result = unsafe { QueryFullProcessImageNameW(handle, 0, buf.as_mut_ptr(), &mut size) };

    // SAFETY: handle is valid from OpenProcess above.
    unsafe { CloseHandle(handle) };

    if result != 0 && size > 0 {
        buf.truncate(size as usize);
        Some(OsString::from_wide(&buf).to_string_lossy().into_owned())
    } else {
        None
    }
}

fn get_memory(handle: *mut std::ffi::c_void) -> Option<u64> {
    #[repr(C)]
    struct ProcessMemoryCounters {
        cb: u32,
        page_fault_count: u32,
        peak_working_set_size: usize,
        working_set_size: usize,
        quota_peak_paged_pool_usage: usize,
        quota_paged_pool_usage: usize,
        quota_peak_non_paged_pool_usage: usize,
        quota_non_paged_pool_usage: usize,
        pagefile_usage: usize,
        peak_pagefile_usage: usize,
    }

    extern "system" {
        fn K32GetProcessMemoryInfo(
            hProcess: *mut std::ffi::c_void,
            ppsmemCounters: *mut ProcessMemoryCounters,
            cb: u32,
        ) -> i32;
    }

    // SAFETY: ProcessMemoryCounters is repr(C) where all-zeros is valid.
    let mut counters: ProcessMemoryCounters = unsafe { std::mem::zeroed() };
    counters.cb = std::mem::size_of::<ProcessMemoryCounters>() as u32;

    // SAFETY: handle is valid; counters.cb is set correctly.
    let result = unsafe { K32GetProcessMemoryInfo(handle, &mut counters, counters.cb) };

    if result != 0 {
        Some((counters.working_set_size / 1024) as u64)
    } else {
        None
    }
}

fn calculate_cpu_usage(pid: u32, handle: *mut std::ffi::c_void) -> f32 {
    #[repr(C)]
    struct Filetime {
        low: u32,
        high: u32,
    }

    extern "system" {
        fn GetProcessTimes(
            hProcess: *mut std::ffi::c_void,
            lpCreationTime: *mut Filetime,
            lpExitTime: *mut Filetime,
            lpKernelTime: *mut Filetime,
            lpUserTime: *mut Filetime,
        ) -> i32;
    }

    let mut creation = Filetime { low: 0, high: 0 };
    let mut exit = Filetime { low: 0, high: 0 };
    let mut kernel = Filetime { low: 0, high: 0 };
    let mut user = Filetime { low: 0, high: 0 };

    // SAFETY: handle is valid; all Filetime output pointers are initialized.
    let result = unsafe {
        GetProcessTimes(handle, &mut creation, &mut exit, &mut kernel, &mut user)
    };

    if result == 0 {
        return 0.0;
    }

    let kernel_time = ((kernel.high as u64) << 32) | (kernel.low as u64);
    let user_time = ((user.high as u64) << 32) | (user.low as u64);
    let total_time = kernel_time + user_time;

    let num_cpus = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1);

    if let Ok(mut cache) = cpu_cache().write() {
        let now = Instant::now();

        if let Some((prev_total, prev_time)) = cache.get(&pid) {
            let time_delta = now.duration_since(*prev_time).as_secs_f32();
            if time_delta > 0.0 {
                let cpu_delta = total_time.saturating_sub(*prev_total) as f32;
                // Convert 100-nanosecond intervals to percentage
                let cpu_percent = (cpu_delta / 10_000_000.0 / time_delta) * 100.0;
                cache.insert(pid, (total_time, now));
                return cpu_percent.min(100.0 * num_cpus as f32);
            }
        }

        if cache.len() >= CPU_CACHE_MAX_SIZE {
            let cutoff = Instant::now() - std::time::Duration::from_secs(60);
            cache.retain(|_, (_, ts)| *ts > cutoff);
        }
        cache.insert(pid, (total_time, now));
    }

    0.0
}

fn get_start_time(handle: *mut std::ffi::c_void) -> Option<String> {
    #[repr(C)]
    struct Filetime {
        low: u32,
        high: u32,
    }

    extern "system" {
        fn GetProcessTimes(
            hProcess: *mut std::ffi::c_void,
            lpCreationTime: *mut Filetime,
            lpExitTime: *mut Filetime,
            lpKernelTime: *mut Filetime,
            lpUserTime: *mut Filetime,
        ) -> i32;
    }

    let mut creation = Filetime { low: 0, high: 0 };
    let mut exit = Filetime { low: 0, high: 0 };
    let mut kernel = Filetime { low: 0, high: 0 };
    let mut user = Filetime { low: 0, high: 0 };

    // SAFETY: handle is valid; all Filetime output pointers are initialized.
    let result = unsafe {
        GetProcessTimes(handle, &mut creation, &mut exit, &mut kernel, &mut user)
    };

    if result == 0 {
        return None;
    }

    let creation_time = ((creation.high as u64) << 32) | (creation.low as u64);

    // Difference between 1601 and 1970 in 100-nanosecond intervals
    const EPOCH_DIFF: u64 = 116444736000000000;

    if creation_time > EPOCH_DIFF {
        let unix_time = (creation_time - EPOCH_DIFF) / 10_000_000;
        let start = std::time::UNIX_EPOCH + std::time::Duration::from_secs(unix_time);
        if let Ok(elapsed) = std::time::SystemTime::now().duration_since(start) {
            return Some(super::format_duration(elapsed));
        }
    }

    None
}
