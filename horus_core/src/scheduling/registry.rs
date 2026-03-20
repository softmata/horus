//! Live node registry in shared memory.
//!
//! Each scheduler creates a `SchedulerRegistry` file in the shared memory
//! scheduler directory (path from `horus_sys::shm::shm_scheduler_dir()`)
//! containing atomic `NodeSlot` entries. The scheduler updates these on
//! every tick (~5ns per node). External tools mmap the file and read live
//! metrics directly.

use crate::memory::platform::shm_scheduler_dir;
use memmap2::MmapMut;
use std::fs::{self, OpenOptions};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::Mutex;

/// Magic number for registry file validation.
pub const REGISTRY_MAGIC: u64 = 0x484F525553524547; // "HORUSREG"

/// Registry format version.
pub const REGISTRY_VERSION: u32 = 1;

/// Maximum number of nodes per scheduler registry.
pub const MAX_REGISTRY_NODES: usize = 64;

/// Size of the registry header (1 cache line).
const HEADER_SIZE: usize = 64;

/// Size of each node slot (3 cache lines).
const SLOT_SIZE: usize = 192;

/// Total file size: header + MAX_REGISTRY_NODES * SLOT_SIZE.
const REGISTRY_FILE_SIZE: usize = HEADER_SIZE + MAX_REGISTRY_NODES * SLOT_SIZE;

/// Registry header — first 64 bytes of the file.
#[repr(C, align(64))]
pub struct RegistryHeader {
    /// Magic number (`REGISTRY_MAGIC`) — set last to signal initialization.
    pub magic: u64,
    /// Format version (`REGISTRY_VERSION`).
    pub version: u32,
    /// Number of active node slots.
    pub node_count: AtomicU32,
    /// PID of the scheduler process.
    pub scheduler_pid: u32,
    /// Reserved.
    pub _pad: [u8; 40],
}

const _: () = assert!(std::mem::size_of::<RegistryHeader>() == 64);

/// Per-node slot with atomic fields for live metrics.
///
/// 192 bytes = 3 cache lines. Written by the scheduler on every tick,
/// read by external tools via mmap.
#[repr(C, align(64))]
pub struct NodeSlot {
    // === Cache line 1: Identity (64 bytes) ===
    /// Node name, null-terminated.
    pub name: [u8; 48],
    /// Node state: 0=Running, 1=Paused, 2=Stopped, 3=Error.
    pub state: AtomicU8,
    /// Node health: 0=Healthy, 1=Warning, 2=Unhealthy, 3=Isolated.
    pub health: AtomicU8,
    /// Execution class (Rt=0, Compute=1, Event=2, AsyncIo=3, BestEffort=4).
    pub execution_class: u8,
    /// Execution order.
    pub order: u8,
    /// Rate in Hz × 100 (fixed-point, e.g. 10000 = 100.00 Hz).
    pub rate_hz_x100: AtomicU32,
    /// Padding to 64 bytes.
    pub _pad0: [u8; 8],

    // === Cache line 2: Counters (64 bytes) ===
    /// Total ticks executed.
    pub tick_count: AtomicU64,
    /// Total errors.
    pub error_count: AtomicU32,
    /// Budget overruns.
    pub budget_misses: AtomicU32,
    /// Deadline misses.
    pub deadline_misses: AtomicU32,
    /// Padding.
    pub _pad1: [u8; 36],

    // === Cache line 3: Timing (64 bytes) ===
    /// Duration of most recent tick (nanoseconds).
    pub last_tick_ns: AtomicU64,
    /// Rolling average tick duration (nanoseconds).
    pub avg_tick_ns: AtomicU64,
    /// Worst-case tick duration since start (nanoseconds).
    pub max_tick_ns: AtomicU64,
    /// Padding.
    pub _pad2: [u8; 40],
}

const _: () = assert!(std::mem::size_of::<NodeSlot>() == 192);

/// Non-atomic snapshot of a `NodeSlot` for external consumers.
#[derive(Debug, Clone)]
pub struct NodeSlotSnapshot {
    pub name: String,
    pub state: u8,
    pub health: u8,
    pub execution_class: u8,
    pub order: u8,
    pub rate_hz: f64,
    pub tick_count: u64,
    pub error_count: u32,
    pub budget_misses: u32,
    pub deadline_misses: u32,
    pub last_tick_ns: u64,
    pub avg_tick_ns: u64,
    pub max_tick_ns: u64,
}

/// Live node registry backed by a shared-memory file.
pub struct SchedulerRegistry {
    mmap: Mutex<MmapMut>,
    path: PathBuf,
}

impl SchedulerRegistry {
    /// Create or open the registry for a scheduler.
    ///
    /// Creates the file at `shm_scheduler_dir()/{scheduler_name}`.
    pub fn open(scheduler_name: &str) -> crate::error::HorusResult<Self> {
        let dir = shm_scheduler_dir();
        horus_sys::fs::create_dir_secure(&dir)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        let path = dir.join(scheduler_name);
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&path)?;

        let meta = file.metadata()?;
        if meta.len() == 0 {
            file.set_len(REGISTRY_FILE_SIZE as u64)?;
        }

        // SAFETY: file is valid, size set to REGISTRY_FILE_SIZE.
        let mmap = unsafe {
            MmapMut::map_mut(&file).map_err(|e| {
                crate::error::HorusError::Memory(crate::error::MemoryError::MmapFailed {
                    reason: e.to_string(),
                })
            })?
        };

        let registry = Self {
            mmap: Mutex::new(mmap),
            path,
        };

        // Initialize header
        registry.init_header();

        Ok(registry)
    }

    fn init_header(&self) {
        let guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
        let base = guard.as_ptr() as *mut u8;
        // SAFETY: mmap is at least REGISTRY_FILE_SIZE bytes, HEADER_SIZE=64.
        unsafe {
            // Version first
            std::ptr::write_unaligned(base.add(8) as *mut u32, REGISTRY_VERSION);
            // Node count = 0
            (*(base.add(12) as *const AtomicU32)).store(0, Ordering::Release);
            // Scheduler PID
            std::ptr::write_unaligned(base.add(16) as *mut u32, std::process::id());
            // Magic LAST (signals header ready)
            std::sync::atomic::fence(Ordering::Release);
            std::ptr::write_unaligned(base as *mut u64, REGISTRY_MAGIC);
        }
    }

    /// Register a node and return its slot index.
    pub fn register_node(&self, name: &str, order: u8, rate_hz: f64, execution_class: u8) -> usize {
        let mut guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
        let base = guard.as_mut_ptr();

        // SAFETY: mmap covers REGISTRY_FILE_SIZE bytes.
        let count =
            unsafe { (*(base.add(12) as *const AtomicU32)).load(Ordering::Acquire) } as usize;

        if count >= MAX_REGISTRY_NODES {
            return count.min(MAX_REGISTRY_NODES - 1);
        }

        let slot_offset = HEADER_SIZE + count * SLOT_SIZE;

        // SAFETY: slot_offset + SLOT_SIZE <= REGISTRY_FILE_SIZE (count < MAX).
        unsafe {
            let slot = base.add(slot_offset);

            // Write name (null-terminated, truncated)
            let name_bytes = name.as_bytes();
            let copy_len = name_bytes.len().min(47);
            std::ptr::write_bytes(slot, 0, 48); // clear name field
            std::ptr::copy_nonoverlapping(name_bytes.as_ptr(), slot, copy_len);

            // State = Running (0)
            (*(slot.add(48) as *const AtomicU8)).store(0, Ordering::Release);
            // Health = Healthy (0)
            (*(slot.add(49) as *const AtomicU8)).store(0, Ordering::Release);
            // Execution class
            *slot.add(50) = execution_class;
            // Order
            *slot.add(51) = order;
            // Rate
            (*(slot.add(52) as *const AtomicU32))
                .store((rate_hz * 100.0) as u32, Ordering::Release);

            // Counters = 0
            (*(slot.add(64) as *const AtomicU64)).store(0, Ordering::Release); // tick_count
            (*(slot.add(72) as *const AtomicU32)).store(0, Ordering::Release); // error_count
            (*(slot.add(76) as *const AtomicU32)).store(0, Ordering::Release); // budget_misses
            (*(slot.add(80) as *const AtomicU32)).store(0, Ordering::Release); // deadline_misses

            // Timing = 0
            (*(slot.add(128) as *const AtomicU64)).store(0, Ordering::Release); // last_tick_ns
            (*(slot.add(136) as *const AtomicU64)).store(0, Ordering::Release); // avg_tick_ns
            (*(slot.add(144) as *const AtomicU64)).store(0, Ordering::Release); // max_tick_ns

            // Increment node count
            (*(base.add(12) as *const AtomicU32)).fetch_add(1, Ordering::Release);
        }

        count
    }

    /// Update a node's live metrics (called by scheduler on every tick).
    ///
    /// All writes are `Relaxed` — counters and timing don't need ordering.
    pub fn update_node(
        &self,
        slot_idx: usize,
        health: u8,
        tick_count: u64,
        error_count: u32,
        budget_misses: u32,
        deadline_misses: u32,
        last_tick_ns: u64,
        avg_tick_ns: u64,
        max_tick_ns: u64,
    ) {
        if slot_idx >= MAX_REGISTRY_NODES {
            return;
        }
        let guard = self.mmap.lock().unwrap_or_else(|e| e.into_inner());
        let base = guard.as_ptr() as *mut u8;
        let slot_offset = HEADER_SIZE + slot_idx * SLOT_SIZE;

        // SAFETY: slot_idx < MAX, so slot_offset + SLOT_SIZE <= REGISTRY_FILE_SIZE.
        unsafe {
            let slot = base.add(slot_offset);
            (*(slot.add(49) as *const AtomicU8)).store(health, Ordering::Relaxed);
            (*(slot.add(64) as *const AtomicU64)).store(tick_count, Ordering::Relaxed);
            (*(slot.add(72) as *const AtomicU32)).store(error_count, Ordering::Relaxed);
            (*(slot.add(76) as *const AtomicU32)).store(budget_misses, Ordering::Relaxed);
            (*(slot.add(80) as *const AtomicU32)).store(deadline_misses, Ordering::Relaxed);
            (*(slot.add(128) as *const AtomicU64)).store(last_tick_ns, Ordering::Relaxed);
            (*(slot.add(136) as *const AtomicU64)).store(avg_tick_ns, Ordering::Relaxed);
            (*(slot.add(144) as *const AtomicU64)).store(max_tick_ns, Ordering::Relaxed);
        }
    }

    /// Remove the registry file (called on scheduler shutdown).
    pub fn remove(&self) -> std::io::Result<()> {
        if self.path.exists() {
            fs::remove_file(&self.path)?;
        }
        Ok(())
    }

    /// Read all active node slots from a registry file (static reader for external tools).
    ///
    /// Returns `None` if the file doesn't exist or has an invalid header.
    pub fn read_all_slots(scheduler_name: &str) -> Option<Vec<NodeSlotSnapshot>> {
        let path = shm_scheduler_dir().join(scheduler_name);
        Self::read_all_slots_from_path(&path)
    }

    /// Read all active node slots from a registry file at the given path.
    pub fn read_all_slots_from_path(path: &std::path::Path) -> Option<Vec<NodeSlotSnapshot>> {
        use memmap2::MmapOptions;

        let file = std::fs::File::open(path).ok()?;
        let meta = file.metadata().ok()?;
        if (meta.len() as usize) < HEADER_SIZE {
            return None;
        }
        // SAFETY: read-only mmap of a valid file.
        let mmap = unsafe { MmapOptions::new().map(&file).ok()? };
        let base = mmap.as_ptr();

        // Validate magic
        let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
        if magic != REGISTRY_MAGIC {
            return None;
        }

        let node_count =
            unsafe { (*(base.add(12) as *const AtomicU32)).load(Ordering::Acquire) } as usize;
        let node_count = node_count.min(MAX_REGISTRY_NODES);

        let mut slots = Vec::with_capacity(node_count);
        for i in 0..node_count {
            let slot_offset = HEADER_SIZE + i * SLOT_SIZE;
            if slot_offset + SLOT_SIZE > mmap.len() {
                break;
            }

            // SAFETY: bounds checked above.
            unsafe {
                let slot = base.add(slot_offset);

                // Read name
                let name_bytes = std::slice::from_raw_parts(slot, 48);
                let end = name_bytes.iter().position(|&b| b == 0).unwrap_or(48);
                let name = std::str::from_utf8(&name_bytes[..end])
                    .unwrap_or("")
                    .to_string();

                if name.is_empty() {
                    continue;
                }

                let state = (*(slot.add(48) as *const AtomicU8)).load(Ordering::Relaxed);
                let health = (*(slot.add(49) as *const AtomicU8)).load(Ordering::Relaxed);
                let execution_class = *slot.add(50);
                let order = *slot.add(51);
                let rate_hz_x100 = (*(slot.add(52) as *const AtomicU32)).load(Ordering::Relaxed);

                let tick_count = (*(slot.add(64) as *const AtomicU64)).load(Ordering::Relaxed);
                let error_count = (*(slot.add(72) as *const AtomicU32)).load(Ordering::Relaxed);
                let budget_misses = (*(slot.add(76) as *const AtomicU32)).load(Ordering::Relaxed);
                let deadline_misses = (*(slot.add(80) as *const AtomicU32)).load(Ordering::Relaxed);

                let last_tick_ns = (*(slot.add(128) as *const AtomicU64)).load(Ordering::Relaxed);
                let avg_tick_ns = (*(slot.add(136) as *const AtomicU64)).load(Ordering::Relaxed);
                let max_tick_ns = (*(slot.add(144) as *const AtomicU64)).load(Ordering::Relaxed);

                slots.push(NodeSlotSnapshot {
                    name,
                    state,
                    health,
                    execution_class,
                    order,
                    rate_hz: rate_hz_x100 as f64 / 100.0,
                    tick_count,
                    error_count,
                    budget_misses,
                    deadline_misses,
                    last_tick_ns,
                    avg_tick_ns,
                    max_tick_ns,
                });
            }
        }

        Some(slots)
    }

    /// Scan the scheduler directory and read all registry files.
    pub fn read_all_registries() -> Vec<(String, Vec<NodeSlotSnapshot>)> {
        let dir = shm_scheduler_dir();
        if !dir.exists() {
            return Vec::new();
        }
        let mut results = Vec::new();
        if let Ok(entries) = fs::read_dir(&dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_file() {
                    if let Some(slots) = Self::read_all_slots_from_path(&path) {
                        let name = path
                            .file_name()
                            .and_then(|n| n.to_str())
                            .unwrap_or("")
                            .to_string();
                        results.push((name, slots));
                    }
                }
            }
        }
        results
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_header_and_slot_sizes() {
        assert_eq!(std::mem::size_of::<RegistryHeader>(), 64);
        assert_eq!(std::mem::size_of::<NodeSlot>(), 192);
        assert_eq!(
            REGISTRY_FILE_SIZE,
            64 + 64 * 192 // 12352 bytes
        );
    }

    #[test]
    fn node_slot_snapshot_default() {
        let snap = NodeSlotSnapshot {
            name: "test".to_string(),
            state: 0,
            health: 0,
            execution_class: 0,
            order: 0,
            rate_hz: 100.0,
            tick_count: 0,
            error_count: 0,
            budget_misses: 0,
            deadline_misses: 0,
            last_tick_ns: 0,
            avg_tick_ns: 0,
            max_tick_ns: 0,
        };
        assert_eq!(snap.name, "test");
        assert_eq!(snap.rate_hz, 100.0);
    }

    // ── Helpers ────────────────────────────────────────────────────────

    fn test_name(id: &str) -> String {
        format!("test_reg_{}_{}", std::process::id(), id)
    }

    // ── Lifecycle ──────────────────────────────────────────────────────

    #[test]
    fn open_creates_file() {
        let name = test_name("open");
        let reg = SchedulerRegistry::open(&name).expect("open");
        let path = shm_scheduler_dir().join(&name);
        assert!(path.exists(), "registry file should exist after open");
        reg.remove().expect("cleanup");
    }

    #[test]
    fn open_empty_registry_readable() {
        let name = test_name("empty");
        let reg = SchedulerRegistry::open(&name).expect("open");
        let slots = SchedulerRegistry::read_all_slots(&name).expect("should read valid header");
        assert!(slots.is_empty(), "no nodes registered yet");
        reg.remove().expect("cleanup");
    }

    #[test]
    fn register_single_node_roundtrip() {
        let name = test_name("single");
        let reg = SchedulerRegistry::open(&name).expect("open");
        let idx = reg.register_node("motor_ctrl", 0, 100.0, 0);
        assert_eq!(idx, 0, "first node should get slot 0");

        let slots = SchedulerRegistry::read_all_slots(&name).expect("read");
        assert_eq!(slots.len(), 1);
        assert_eq!(slots[0].name, "motor_ctrl");
        assert_eq!(slots[0].order, 0);
        assert_eq!(slots[0].rate_hz, 100.0);
        assert_eq!(slots[0].execution_class, 0);
        assert_eq!(slots[0].state, 0); // Running
        assert_eq!(slots[0].health, 0); // Healthy
        assert_eq!(slots[0].tick_count, 0);
        assert_eq!(slots[0].error_count, 0);

        reg.remove().expect("cleanup");
    }

    #[test]
    fn register_multiple_nodes() {
        let name = test_name("multi");
        let reg = SchedulerRegistry::open(&name).expect("open");

        let idx0 = reg.register_node("motor", 0, 100.0, 0);
        let idx1 = reg.register_node("sensor", 1, 200.0, 1);
        let idx2 = reg.register_node("planner", 5, 10.0, 2);
        let idx3 = reg.register_node("logger", 10, 1.0, 4);
        let idx4 = reg.register_node("telemetry", 20, 0.5, 3);

        assert_eq!(idx0, 0);
        assert_eq!(idx1, 1);
        assert_eq!(idx2, 2);
        assert_eq!(idx3, 3);
        assert_eq!(idx4, 4);

        let slots = SchedulerRegistry::read_all_slots(&name).expect("read");
        assert_eq!(slots.len(), 5);
        assert_eq!(slots[0].name, "motor");
        assert_eq!(slots[1].name, "sensor");
        assert_eq!(slots[2].name, "planner");
        assert_eq!(slots[3].name, "logger");
        assert_eq!(slots[4].name, "telemetry");
        assert_eq!(slots[1].rate_hz, 200.0);
        assert_eq!(slots[2].order, 5);
        assert_eq!(slots[4].execution_class, 3);

        reg.remove().expect("cleanup");
    }

    #[test]
    fn update_node_changes_visible() {
        let name = test_name("update");
        let reg = SchedulerRegistry::open(&name).expect("open");
        let idx = reg.register_node("sensor", 1, 200.0, 1);

        reg.update_node(
            idx, 2,         // health: Unhealthy
            5000,      // tick_count
            3,         // error_count
            2,         // budget_misses
            1,         // deadline_misses
            1_200_000, // last_tick_ns
            1_100_000, // avg_tick_ns
            2_000_000, // max_tick_ns
        );

        let slots = SchedulerRegistry::read_all_slots(&name).expect("read");
        assert_eq!(slots[0].health, 2);
        assert_eq!(slots[0].tick_count, 5000);
        assert_eq!(slots[0].error_count, 3);
        assert_eq!(slots[0].budget_misses, 2);
        assert_eq!(slots[0].deadline_misses, 1);
        assert_eq!(slots[0].last_tick_ns, 1_200_000);
        assert_eq!(slots[0].avg_tick_ns, 1_100_000);
        assert_eq!(slots[0].max_tick_ns, 2_000_000);

        reg.remove().expect("cleanup");
    }

    #[test]
    fn update_node_multiple_times() {
        let name = test_name("multi_upd");
        let reg = SchedulerRegistry::open(&name).expect("open");
        let idx = reg.register_node("ctrl", 0, 100.0, 0);

        for tick in 1..=10u64 {
            reg.update_node(idx, 0, tick * 100, 0, 0, 0, 500, 500, 500);
        }

        let slots = SchedulerRegistry::read_all_slots(&name).expect("read");
        assert_eq!(slots[0].tick_count, 1000, "should have latest update");

        reg.remove().expect("cleanup");
    }

    // ── Edge Cases ─────────────────────────────────────────────────────

    #[test]
    fn register_node_name_truncation() {
        let name = test_name("trunc");
        let reg = SchedulerRegistry::open(&name).expect("open");
        let long_name = "X".repeat(60);
        reg.register_node(&long_name, 0, 50.0, 0);

        let slots = SchedulerRegistry::read_all_slots(&name).expect("read");
        assert_eq!(
            slots[0].name.len(),
            47,
            "name should be truncated to 47 chars"
        );
        assert_eq!(slots[0].name, "X".repeat(47));

        reg.remove().expect("cleanup");
    }

    #[test]
    fn update_node_invalid_index_ignored() {
        let name = test_name("bad_idx");
        let reg = SchedulerRegistry::open(&name).expect("open");
        // No nodes registered, update at invalid index — should not panic
        reg.update_node(999, 0, 100, 0, 0, 0, 0, 0, 0);
        reg.update_node(MAX_REGISTRY_NODES, 0, 100, 0, 0, 0, 0, 0, 0);

        let slots = SchedulerRegistry::read_all_slots(&name).expect("read");
        assert!(slots.is_empty(), "no nodes should exist");

        reg.remove().expect("cleanup");
    }

    #[test]
    fn remove_deletes_file() {
        let name = test_name("rm");
        let reg = SchedulerRegistry::open(&name).expect("open");
        reg.register_node("node", 0, 10.0, 0);
        let path = shm_scheduler_dir().join(&name);
        assert!(path.exists());

        reg.remove().expect("remove");
        assert!(!path.exists(), "file should be deleted after remove");
    }

    #[test]
    fn read_nonexistent_returns_none() {
        assert!(
            SchedulerRegistry::read_all_slots("totally_nonexistent_scheduler_name_12345").is_none()
        );
    }

    #[test]
    fn read_invalid_magic_returns_none() {
        let path =
            std::env::temp_dir().join(format!("horus_reg_bad_magic_{}.bin", std::process::id()));
        std::fs::write(&path, &[0u8; REGISTRY_FILE_SIZE]).expect("write");
        assert!(
            SchedulerRegistry::read_all_slots_from_path(&path).is_none(),
            "should return None for invalid magic"
        );
        let _ = std::fs::remove_file(&path);
    }

    // ── Cross-process simulation ───────────────────────────────────────

    #[test]
    fn read_all_slots_from_path_cross_process() {
        // Simulates: scheduler writes via open/register/update,
        // external tool reads via read_all_slots_from_path (separate mmap)
        let name = test_name("xproc");
        let reg = SchedulerRegistry::open(&name).expect("open");

        reg.register_node("motor", 0, 100.0, 0);
        reg.register_node("sensor", 1, 200.0, 1);
        reg.update_node(0, 0, 10000, 0, 0, 0, 800_000, 750_000, 1_500_000);
        reg.update_node(1, 1, 20000, 5, 3, 1, 400_000, 380_000, 900_000);

        // Read via separate path (simulates external tool opening the file)
        let path = shm_scheduler_dir().join(&name);
        let slots = SchedulerRegistry::read_all_slots_from_path(&path).expect("external read");

        assert_eq!(slots.len(), 2);
        assert_eq!(slots[0].name, "motor");
        assert_eq!(slots[0].tick_count, 10000);
        assert_eq!(slots[0].avg_tick_ns, 750_000);
        assert_eq!(slots[1].name, "sensor");
        assert_eq!(slots[1].health, 1); // Warning
        assert_eq!(slots[1].error_count, 5);
        assert_eq!(slots[1].budget_misses, 3);
        assert_eq!(slots[1].deadline_misses, 1);

        reg.remove().expect("cleanup");
    }

    #[test]
    fn read_all_registries_finds_files() {
        let name1 = test_name("scan_a");
        let name2 = test_name("scan_b");
        let reg1 = SchedulerRegistry::open(&name1).expect("open 1");
        let reg2 = SchedulerRegistry::open(&name2).expect("open 2");

        reg1.register_node("node_a", 0, 50.0, 0);
        reg2.register_node("node_b", 0, 60.0, 0);

        let all = SchedulerRegistry::read_all_registries();
        let found_a = all
            .iter()
            .any(|(n, slots)| n == &name1 && slots.len() == 1 && slots[0].name == "node_a");
        let found_b = all
            .iter()
            .any(|(n, slots)| n == &name2 && slots.len() == 1 && slots[0].name == "node_b");

        assert!(found_a, "should find registry '{}'", name1);
        assert!(found_b, "should find registry '{}'", name2);

        reg1.remove().expect("cleanup 1");
        reg2.remove().expect("cleanup 2");
    }
}
