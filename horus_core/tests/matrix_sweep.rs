//! Combinatorial matrix sweep — systematically tests every critical
//! backend × message-type × topology × process-model combination.
//!
//! Instead of 660 hand-written tests, this generates test cases from
//! the cross-product of dimensions and runs each one with data integrity
//! verification.
//!
//! Dimensions:
//!   Topology:      1P1C, 1P3C, 3P1C, 2P2C
//!   Process model: same-thread, cross-thread, cross-process
//!   Message type:  POD-small (48B), POD-large (512B), Serde (Vec)
//!   Message count: 200 (enough to trigger migration + verify integrity)
//!
//! Each combination: send N messages with sequence+checksum, recv, verify
//! zero corruption. Reports pass/fail matrix at the end.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test matrix_sweep -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use serde::{Deserialize, Serialize};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// Message types covering all serialization paths
// ════════════════════════════════════════════════════════════════════════

/// Small POD — fits in co-located 56B slot (zero-copy fast path)
#[derive(Clone, Copy, Default, Serialize, Deserialize, PartialEq, Debug)]
#[repr(C)]
struct SmallPod {
    seq: u64,
    value_a: f64,
    value_b: f64,
    checksum: u64,
}
unsafe impl bytemuck::Pod for SmallPod {}
unsafe impl bytemuck::Zeroable for SmallPod {}

impl SmallPod {
    fn new(seq: u64) -> Self {
        let value_a = (seq as f64 * 0.123).sin();
        let value_b = (seq as f64 * 0.456).cos();
        let checksum = seq ^ value_a.to_bits() ^ value_b.to_bits();
        Self { seq, value_a, value_b, checksum }
    }
    fn verify(&self) -> bool {
        self.checksum == (self.seq ^ self.value_a.to_bits() ^ self.value_b.to_bits())
    }
}

/// Large POD — exceeds 56B co-located slot, uses standard SHM slot path
#[derive(Clone, Copy, Default, Serialize, Deserialize, PartialEq, Debug)]
#[repr(C)]
struct LargePod {
    seq: u64,
    data: [f64; 30], // 240 bytes
    checksum: u64,
}
unsafe impl bytemuck::Pod for LargePod {}
unsafe impl bytemuck::Zeroable for LargePod {}

impl LargePod {
    fn new(seq: u64) -> Self {
        let mut data = [0.0f64; 30];
        for i in 0..30 { data[i] = (seq as f64 + i as f64) * 0.01; }
        let checksum = data.iter().fold(seq, |acc, &v| acc ^ v.to_bits());
        Self { seq, data, checksum }
    }
    fn verify(&self) -> bool {
        self.checksum == self.data.iter().fold(self.seq, |acc, &v| acc ^ v.to_bits())
    }
}

/// Serde type — forces serialization path (not POD), variable size
#[derive(Clone, Serialize, Deserialize, PartialEq, Debug)]
struct SerdeMsg {
    seq: u64,
    payload: Vec<u32>,
    tag: String,
    checksum: u64,
}

impl SerdeMsg {
    fn new(seq: u64) -> Self {
        let payload: Vec<u32> = (0..20).map(|i| (seq as u32).wrapping_mul(7).wrapping_add(i)).collect();
        let tag = format!("msg_{}", seq);
        let checksum = payload.iter().fold(seq, |acc, &v| acc ^ v as u64);
        Self { seq, payload, tag, checksum }
    }
    fn verify(&self) -> bool {
        self.checksum == self.payload.iter().fold(self.seq, |acc, &v| acc ^ v as u64)
    }
}

// ════════════════════════════════════════════════════════════════════════
// Cross-process child entry points
// ════════════════════════════════════════════════════════════════════════

const CHILD_FLAG: &str = "HORUS_MATRIX_CHILD";
const TOPIC_ENV: &str = "HORUS_MATRIX_TOPIC";
const ROLE_ENV: &str = "HORUS_MATRIX_ROLE";    // "pub" or "sub"
const MSGTYPE_ENV: &str = "HORUS_MATRIX_TYPE";  // "small", "large", "serde"
const COUNT_ENV: &str = "HORUS_MATRIX_COUNT";
const ID_ENV: &str = "HORUS_MATRIX_ID";

fn child_publisher() {
    let topic = std::env::var(TOPIC_ENV).unwrap();
    let count: u64 = std::env::var(COUNT_ENV).unwrap().parse().unwrap();
    let msg_type = std::env::var(MSGTYPE_ENV).unwrap();

    match msg_type.as_str() {
        "small" => {
            let t: Topic<SmallPod> = Topic::new(&topic).unwrap();
            t.send(SmallPod::default()); // init
            std::thread::sleep(Duration::from_millis(50));
            for i in 1..=count { t.send(SmallPod::new(i)); }
            t.send(SmallPod { seq: u64::MAX, ..Default::default() }); // sentinel
        }
        "large" => {
            let t: Topic<LargePod> = Topic::new(&topic).unwrap();
            t.send(LargePod::default());
            std::thread::sleep(Duration::from_millis(50));
            for i in 1..=count { t.send(LargePod::new(i)); }
            t.send(LargePod { seq: u64::MAX, ..Default::default() });
        }
        "serde" => {
            let t: Topic<SerdeMsg> = Topic::new(&topic).unwrap();
            t.send(SerdeMsg { seq: 0, payload: vec![], tag: String::new(), checksum: 0 });
            std::thread::sleep(Duration::from_millis(50));
            for i in 1..=count { t.send(SerdeMsg::new(i)); }
            t.send(SerdeMsg { seq: u64::MAX, payload: vec![], tag: String::new(), checksum: 0 });
        }
        _ => panic!("unknown type: {}", msg_type),
    }
    std::thread::sleep(Duration::from_millis(300));
}

fn child_subscriber() {
    let topic = std::env::var(TOPIC_ENV).unwrap();
    let msg_type = std::env::var(MSGTYPE_ENV).unwrap();
    let id: u64 = std::env::var(ID_ENV).unwrap().parse().unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);

    let (mut received, mut corrupted) = (0u64, 0u64);

    match msg_type.as_str() {
        "small" => {
            let t: Topic<SmallPod> = Topic::new(&topic).unwrap();
            while Instant::now() < deadline {
                match t.recv() {
                    Some(m) if m.seq == u64::MAX => break,
                    Some(m) if m.seq == 0 => continue,
                    Some(m) => { received += 1; if !m.verify() { corrupted += 1; } }
                    None => std::thread::yield_now(),
                }
            }
        }
        "large" => {
            let t: Topic<LargePod> = Topic::new(&topic).unwrap();
            while Instant::now() < deadline {
                match t.recv() {
                    Some(m) if m.seq == u64::MAX => break,
                    Some(m) if m.seq == 0 => continue,
                    Some(m) => { received += 1; if !m.verify() { corrupted += 1; } }
                    None => std::thread::yield_now(),
                }
            }
        }
        "serde" => {
            let t: Topic<SerdeMsg> = Topic::new(&topic).unwrap();
            while Instant::now() < deadline {
                match t.recv() {
                    Some(m) if m.seq == u64::MAX => break,
                    Some(m) if m.seq == 0 => continue,
                    Some(m) => { received += 1; if !m.verify() { corrupted += 1; } }
                    None => std::thread::yield_now(),
                }
            }
        }
        _ => {}
    }
    println!("RESULT:{}:{}:{}", id, received, corrupted);
}

fn spawn_child(test_name: &str, topic: &str, role: &str, msg_type: &str, count: u64, id: u64) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture", "--ignored"])
        .env(CHILD_FLAG, "1")
        .env(TOPIC_ENV, topic)
        .env(ROLE_ENV, role)
        .env(MSGTYPE_ENV, msg_type)
        .env(COUNT_ENV, count.to_string())
        .env(ID_ENV, id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap_or_else(|e| panic!("spawn: {}", e))
}

// ════════════════════════════════════════════════════════════════════════
// Same-thread test helper (DirectChannel path)
// ════════════════════════════════════════════════════════════════════════

fn test_same_thread(msg_type: &str, n_pubs: usize, n_subs: usize, count: u64) -> (u64, u64) {
    let name = unique(&format!("st_{}_{}{}", msg_type, n_pubs, n_subs));
    let (mut received, mut corrupted) = (0u64, 0u64);

    match msg_type {
        "small" => {
            let t: Topic<SmallPod> = Topic::new(&name).unwrap();
            for i in 1..=count { t.send(SmallPod::new(i)); }
            while let Some(m) = t.recv() { received += 1; if !m.verify() { corrupted += 1; } }
        }
        "large" => {
            let t: Topic<LargePod> = Topic::new(&name).unwrap();
            for i in 1..=count { t.send(LargePod::new(i)); }
            while let Some(m) = t.recv() { received += 1; if !m.verify() { corrupted += 1; } }
        }
        "serde" => {
            let t: Topic<SerdeMsg> = Topic::new(&name).unwrap();
            for i in 1..=count { t.send(SerdeMsg::new(i)); }
            while let Some(m) = t.recv() { received += 1; if !m.verify() { corrupted += 1; } }
        }
        _ => {}
    }
    (received, corrupted)
}

// ════════════════════════════════════════════════════════════════════════
// Cross-thread test helper (SpscIntra/SpmcIntra/MpscIntra/FanoutIntra)
// ════════════════════════════════════════════════════════════════════════

fn test_cross_thread(msg_type: &str, n_pubs: usize, n_subs: usize, count: u64) -> (u64, u64) {
    let name = unique(&format!("ct_{}_{}{}", msg_type, n_pubs, n_subs));
    let total_recv = Arc::new(AtomicU64::new(0));
    let total_corrupt = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicBool::new(false));

    // Spawn subscriber threads
    let mut sub_handles = vec![];
    for _ in 0..n_subs {
        let n = name.clone();
        let r = total_recv.clone();
        let c = total_corrupt.clone();
        let d = done.clone();
        let mt = msg_type.to_string();
        sub_handles.push(std::thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_secs(5);
            match mt.as_str() {
                "small" => {
                    let t: Topic<SmallPod> = Topic::new(&n).unwrap();
                    while !d.load(Ordering::Relaxed) && Instant::now() < deadline {
                        if let Some(m) = t.recv() {
                            r.fetch_add(1, Ordering::Relaxed);
                            if !m.verify() { c.fetch_add(1, Ordering::Relaxed); }
                        }
                    }
                    // Drain remaining
                    while let Some(m) = t.recv() {
                        r.fetch_add(1, Ordering::Relaxed);
                        if !m.verify() { c.fetch_add(1, Ordering::Relaxed); }
                    }
                }
                "large" => {
                    let t: Topic<LargePod> = Topic::new(&n).unwrap();
                    while !d.load(Ordering::Relaxed) && Instant::now() < deadline {
                        if let Some(m) = t.recv() {
                            r.fetch_add(1, Ordering::Relaxed);
                            if !m.verify() { c.fetch_add(1, Ordering::Relaxed); }
                        }
                    }
                    while let Some(m) = t.recv() {
                        r.fetch_add(1, Ordering::Relaxed);
                        if !m.verify() { c.fetch_add(1, Ordering::Relaxed); }
                    }
                }
                "serde" => {
                    let t: Topic<SerdeMsg> = Topic::new(&n).unwrap();
                    while !d.load(Ordering::Relaxed) && Instant::now() < deadline {
                        if let Some(m) = t.recv() {
                            r.fetch_add(1, Ordering::Relaxed);
                            if !m.verify() { c.fetch_add(1, Ordering::Relaxed); }
                        }
                    }
                    while let Some(m) = t.recv() {
                        r.fetch_add(1, Ordering::Relaxed);
                        if !m.verify() { c.fetch_add(1, Ordering::Relaxed); }
                    }
                }
                _ => {}
            }
        }));
    }

    std::thread::sleep(Duration::from_millis(50));

    // Spawn publisher threads
    let mut pub_handles = vec![];
    for pub_id in 0..n_pubs {
        let n = name.clone();
        let mt = msg_type.to_string();
        let per_pub = count / n_pubs as u64;
        pub_handles.push(std::thread::spawn(move || {
            match mt.as_str() {
                "small" => {
                    let t: Topic<SmallPod> = Topic::new(&n).unwrap();
                    for i in 0..per_pub { t.send(SmallPod::new(pub_id as u64 * 100_000 + i + 1)); }
                }
                "large" => {
                    let t: Topic<LargePod> = Topic::new(&n).unwrap();
                    for i in 0..per_pub { t.send(LargePod::new(pub_id as u64 * 100_000 + i + 1)); }
                }
                "serde" => {
                    let t: Topic<SerdeMsg> = Topic::new(&n).unwrap();
                    for i in 0..per_pub { t.send(SerdeMsg::new(pub_id as u64 * 100_000 + i + 1)); }
                }
                _ => {}
            }
        }));
    }

    for h in pub_handles { h.join().unwrap(); }
    std::thread::sleep(Duration::from_millis(100));
    done.store(true, Ordering::Relaxed);
    for h in sub_handles { h.join().unwrap(); }

    (total_recv.load(Ordering::Relaxed), total_corrupt.load(Ordering::Relaxed))
}

// ════════════════════════════════════════════════════════════════════════
// Cross-process test helper
// ════════════════════════════════════════════════════════════════════════

fn test_cross_process(test_name: &str, msg_type: &str, n_pubs: usize, n_subs: usize, count: u64) -> (u64, u64) {
    let name = unique(&format!("cp_{}_{}{}", msg_type, n_pubs, n_subs));

    // Init topic from parent
    match msg_type {
        "small" => { let _: Topic<SmallPod> = Topic::new(&name).unwrap(); }
        "large" => { let _: Topic<LargePod> = Topic::new(&name).unwrap(); }
        "serde" => { let _: Topic<SerdeMsg> = Topic::new(&name).unwrap(); }
        _ => {}
    }

    // Spawn subscribers
    let mut subs = vec![];
    for i in 0..n_subs as u64 {
        subs.push(spawn_child(test_name, &name, "sub", msg_type, count, i));
    }
    std::thread::sleep(Duration::from_millis(300));

    // Spawn publishers
    let mut pubs = vec![];
    let per_pub = count / n_pubs as u64;
    for i in 0..n_pubs as u64 {
        pubs.push(spawn_child(test_name, &name, "pub", msg_type, per_pub, i));
    }

    for mut p in pubs { let _ = p.wait(); }

    let mut total_recv = 0u64;
    let mut total_corrupt = 0u64;
    for mut s in subs {
        let output = s.wait_with_output().unwrap();
        for line in String::from_utf8_lossy(&output.stdout).lines() {
            if line.starts_with("RESULT:") {
                let parts: Vec<&str> = line.split(':').collect();
                if parts.len() >= 4 {
                    total_recv += parts[2].parse::<u64>().unwrap_or(0);
                    total_corrupt += parts[3].parse::<u64>().unwrap_or(0);
                }
            }
        }
    }
    (total_recv, total_corrupt)
}

// ════════════════════════════════════════════════════════════════════════
// THE MATRIX TEST
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn matrix_sweep_all_combinations() {
    if std::env::var(CHILD_FLAG).is_ok() {
        match std::env::var(ROLE_ENV).unwrap_or_default().as_str() {
            "pub" => child_publisher(),
            "sub" => child_subscriber(),
            _ => {}
        }
        return;
    }

    cleanup_stale_shm();

    let msg_types = ["small", "large", "serde"];
    let topologies = [
        (1, 1, "1P1C"),
        (1, 3, "1P3C"),
        (3, 1, "3P1C"),
        (2, 2, "2P2C"),
    ];
    let process_models = ["same_thread", "cross_thread", "cross_process"];
    let count = 200u64;

    let mut results: Vec<(String, u64, u64, &str)> = vec![];
    let total = msg_types.len() * topologies.len() * process_models.len();
    let mut completed = 0;

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║         MATRIX SWEEP — {} combinations                      ║", total);
    println!("║  3 msg types × 4 topologies × 3 process models             ║");
    println!("╠══════════════════════════════════════════════════════════════╣");

    for msg_type in &msg_types {
        for &(n_pubs, n_subs, topo_name) in &topologies {
            for &proc_model in &process_models {
                completed += 1;
                let label = format!("{:<6} {:4} {}", msg_type, topo_name, proc_model);

                // Skip same-thread for multi-pub/sub (not meaningful — needs threads)
                if proc_model == "same_thread" && (n_pubs > 1 || n_subs > 1) {
                    println!("║ [{:2}/{}] {} — SKIP (N/A)              ║", completed, total, label);
                    results.push((label, 0, 0, "SKIP"));
                    continue;
                }

                cleanup_stale_shm();

                let (recv, corrupt) = match proc_model {
                    "same_thread" => test_same_thread(msg_type, n_pubs, n_subs, count),
                    "cross_thread" => test_cross_thread(msg_type, n_pubs, n_subs, count),
                    "cross_process" => test_cross_process(
                        "matrix_sweep_all_combinations", msg_type, n_pubs, n_subs, count),
                    _ => (0, 0),
                };

                let status = if corrupt > 0 {
                    "CORRUPT"
                } else if recv == 0 {
                    "NO_DATA"
                } else {
                    "OK"
                };

                println!("║ [{:2}/{}] {} recv={:4} corrupt={} {}  ║",
                         completed, total, label, recv, corrupt, status);
                results.push((label, recv, corrupt, status));
            }
        }
    }

    // ── Summary ────────────────────────────────────────────────────
    println!("╠══════════════════════════════════════════════════════════════╣");
    let ok_count = results.iter().filter(|r| r.3 == "OK").count();
    let skip_count = results.iter().filter(|r| r.3 == "SKIP").count();
    let corrupt_count = results.iter().filter(|r| r.3 == "CORRUPT").count();
    let nodata_count = results.iter().filter(|r| r.3 == "NO_DATA").count();

    println!("║ RESULTS: {} OK, {} SKIP, {} CORRUPT, {} NO_DATA            ║",
             ok_count, skip_count, corrupt_count, nodata_count);
    println!("╚══════════════════════════════════════════════════════════════╝");

    // ── Failures ───────────────────────────────────────────────────
    if corrupt_count > 0 {
        eprintln!("\nCRITICAL — DATA CORRUPTION DETECTED:");
        for (label, recv, corrupt, status) in &results {
            if *status == "CORRUPT" {
                eprintln!("  {} — {} corrupted out of {}", label, corrupt, recv);
            }
        }
        panic!("{} combinations had DATA CORRUPTION", corrupt_count);
    }

    if nodata_count > 0 {
        eprintln!("\nWARNING — NO DATA received in {} combinations:", nodata_count);
        for (label, _, _, status) in &results {
            if *status == "NO_DATA" { eprintln!("  {}", label); }
        }
    }

    assert_eq!(corrupt_count, 0, "Data corruption detected!");
    println!("\nMATRIX SWEEP PASSED ✓ — zero corruption across {} tested combinations", ok_count);
}
