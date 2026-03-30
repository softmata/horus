//! Rust↔Python full matrix — multiple schedulers, all message types,
//! bidirectional, mixed rates.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test rust_python_matrix -- --ignored --nocapture --test-threads=1

use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_robotics::messages::sensor::*;
use horus_robotics::CmdVel;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

fn python_available() -> bool {
    Command::new("python3")
        .args(["-c", "import horus; print('OK')"])
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).contains("OK"))
        .unwrap_or(false)
}

fn result_dir() -> std::path::PathBuf {
    std::env::temp_dir().join("horus_rp_matrix")
}

fn write_result(key: &str, val: &str) {
    let d = result_dir();
    let _ = std::fs::create_dir_all(&d);
    let _ = std::fs::write(d.join(key), val);
}

fn read_result(key: &str) -> String {
    std::fs::read_to_string(result_dir().join(key)).unwrap_or_default()
}

// ════════════════════════════════════════════════════════════════════════
// TEST 1: All message types — Rust publishes 5 types, Python subscribes
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn rp_all_message_types_rust_to_python() {
    if !python_available() {
        println!("✓ SKIPPED (no horus python)");
        return;
    }
    cleanup_stale_shm();
    let _ = std::fs::remove_dir_all(result_dir());

    let py_code = r#"
import horus, os, sys

counts = {"imu": 0, "cmdvel": 0, "battery": 0, "joint": 0, "scan": 0}
corrupt = [0]

def tick(node):
    for name in ["rp.imu", "rp.cmdvel", "rp.battery", "rp.joint", "rp.scan"]:
        msg = node.recv(name)
        while msg is not None:
            key = name.split(".")[1]
            counts[key] = counts.get(key, 0) + 1
            msg = node.recv(name)

def shutdown(node):
    result_dir = os.environ.get("RESULT_DIR", "/tmp/horus_rp_matrix")
    os.makedirs(result_dir, exist_ok=True)
    for k, v in counts.items():
        with open(os.path.join(result_dir, f"py_{k}"), "w") as f:
            f.write(str(v))

sub = horus.Node("py_all_sub", tick=tick, subs={
    "rp.imu": horus.Imu,
    "rp.cmdvel": horus.CmdVel,
    "rp.battery": horus.BatteryState,
    "rp.joint": horus.JointState,
    "rp.scan": horus.LaserScan,
}, rate=200, shutdown=shutdown)
horus.run(sub, duration=6.0, tick_rate=200)
"#;

    let tmpdir = tempfile::TempDir::new().unwrap();
    let py_file = tmpdir.path().join("all_sub.py");
    std::fs::write(&py_file, py_code).unwrap();

    // Start Python subscriber
    let mut py = Command::new("python3")
        .arg(&py_file)
        .env("RESULT_DIR", result_dir().to_str().unwrap())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();

    std::thread::sleep(Duration::from_secs(2));

    // Rust: 5 publisher nodes in one scheduler
    struct TypedPub<
        T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
    > {
        topic: Option<Topic<T>>,
        name: String,
        make_msg: fn(u64) -> T,
        seq: u64,
        count: Arc<AtomicU64>,
    }
    impl<T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static> Node
        for TypedPub<T>
    {
        fn name(&self) -> &str {
            "typed_pub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.topic = Some(Topic::new(&self.name)?);
            Ok(())
        }
        fn tick(&mut self) {
            let msg = (self.make_msg)(self.seq);
            if let Some(ref t) = self.topic {
                t.send(msg);
            }
            self.seq += 1;
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn make_imu(seq: u64) -> Imu {
        let mut imu = Imu::new();
        imu.linear_acceleration = [0.0, 0.0, 9.81];
        imu.angular_velocity[2] = seq as f64 * 0.001;
        imu
    }
    fn make_cmdvel(_: u64) -> CmdVel {
        CmdVel::new(1.5, 0.5)
    }
    fn make_battery(_: u64) -> BatteryState {
        let mut b = BatteryState::default();
        b.voltage = 12.0;
        b.percentage = 80.0;
        b
    }
    fn make_joint(_: u64) -> JointState {
        JointState::default()
    }
    fn make_scan(_: u64) -> LaserScan {
        LaserScan::default()
    }

    let counts: Vec<Arc<AtomicU64>> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut sched = Scheduler::new().tick_rate(200_u64.hz());
    let _ = sched
        .add(TypedPub::<Imu> {
            topic: None,
            name: "rp.imu".into(),
            make_msg: make_imu,
            seq: 0,
            count: counts[0].clone(),
        })
        .rate(200_u64.hz())
        .order(0)
        .build();
    let _ = sched
        .add(TypedPub::<CmdVel> {
            topic: None,
            name: "rp.cmdvel".into(),
            make_msg: make_cmdvel,
            seq: 0,
            count: counts[1].clone(),
        })
        .rate(100_u64.hz())
        .order(1)
        .build();
    let _ = sched
        .add(TypedPub::<BatteryState> {
            topic: None,
            name: "rp.battery".into(),
            make_msg: make_battery,
            seq: 0,
            count: counts[2].clone(),
        })
        .rate(10_u64.hz())
        .order(2)
        .build();
    let _ = sched
        .add(TypedPub::<JointState> {
            topic: None,
            name: "rp.joint".into(),
            make_msg: make_joint,
            seq: 0,
            count: counts[3].clone(),
        })
        .rate(50_u64.hz())
        .order(3)
        .build();
    let _ = sched
        .add(TypedPub::<LaserScan> {
            topic: None,
            name: "rp.scan".into(),
            make_msg: make_scan,
            seq: 0,
            count: counts[4].clone(),
        })
        .rate(10_u64.hz())
        .order(4)
        .build();
    let _ = sched.run_for(Duration::from_secs(3));

    let _ = py.wait();

    let names = ["imu", "cmdvel", "battery", "joint", "scan"];
    let rust_sent: Vec<u64> = counts.iter().map(|c| c.load(Ordering::Relaxed)).collect();

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  RUST→PYTHON: ALL 5 MESSAGE TYPES                          ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    let mut total_recv = 0u64;
    for (i, name) in names.iter().enumerate() {
        let py_recv: u64 = read_result(&format!("py_{}", name))
            .trim()
            .parse()
            .unwrap_or(0);
        total_recv += py_recv;
        println!(
            "║  {:12} Rust sent: {:5}  Python recv: {:5}  ({:5.1}%)   ║",
            name,
            rust_sent[i],
            py_recv,
            py_recv as f64 / rust_sent[i].max(1) as f64 * 100.0
        );
    }
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert!(
        total_recv > 50,
        "Python should receive messages across types, got {}",
        total_recv
    );
    let types_with_data = names
        .iter()
        .filter(|n| {
            read_result(&format!("py_{}", n))
                .trim()
                .parse::<u64>()
                .unwrap_or(0)
                > 0
        })
        .count();
    assert!(
        types_with_data >= 3,
        "At least 3/5 types should deliver, got {}",
        types_with_data
    );
    println!(
        "✓ rp_all_message_types — {}/5 types delivered, {} total msgs",
        types_with_data, total_recv
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: Python publishes, Rust subscribes — 3 message types
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn rp_python_to_rust_3_types() {
    if !python_available() {
        println!("✓ SKIPPED (no horus python)");
        return;
    }
    cleanup_stale_shm();
    let _ = std::fs::remove_dir_all(result_dir());

    let py_code = r#"
import horus

def tick(node):
    node.send("pr.imu", horus.Imu(0.0, 0.0, 9.81, 0.01, 0.0, 0.0))
    node.send("pr.cmd", horus.CmdVel(2.0, 0.3))
    node.send("pr.bat", horus.BatteryState(12.6, -1.5, 0.0, 0.0, 85.0, 2, 25.0))

pub = horus.Node("py_3pub", tick=tick, pubs={
    "pr.imu": horus.Imu,
    "pr.cmd": horus.CmdVel,
    "pr.bat": horus.BatteryState,
}, rate=100)
horus.run(pub, duration=3.0, tick_rate=100)
"#;

    let tmpdir = tempfile::TempDir::new().unwrap();
    let py_file = tmpdir.path().join("three_pub.py");
    std::fs::write(&py_file, py_code).unwrap();

    let imu_recv = Arc::new(AtomicU64::new(0));
    let cmd_recv = Arc::new(AtomicU64::new(0));
    let bat_recv = Arc::new(AtomicU64::new(0));
    let corrupt = Arc::new(AtomicU64::new(0));
    let running = Arc::new(AtomicBool::new(true));

    let ir = imu_recv.clone();
    let cr = cmd_recv.clone();
    let br = bat_recv.clone();
    let co = corrupt.clone();
    let r = running.clone();

    // Rust subscriber scheduler in thread
    let rust_handle = std::thread::spawn(move || {
        struct MultiSub {
            imu: Option<Topic<Imu>>,
            cmd: Option<Topic<CmdVel>>,
            bat: Option<Topic<BatteryState>>,
            ir: Arc<AtomicU64>,
            cr: Arc<AtomicU64>,
            br: Arc<AtomicU64>,
            co: Arc<AtomicU64>,
        }
        impl Node for MultiSub {
            fn name(&self) -> &str {
                "rust_3sub"
            }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.imu = Some(Topic::new("pr.imu")?);
                self.cmd = Some(Topic::new("pr.cmd")?);
                self.bat = Some(Topic::new("pr.bat")?);
                Ok(())
            }
            fn tick(&mut self) {
                if let Some(ref t) = self.imu {
                    while let Some(imu) = t.recv() {
                        if (imu.linear_acceleration[2] - 9.81).abs() > 0.1 {
                            self.co.fetch_add(1, Ordering::Relaxed);
                        }
                        self.ir.fetch_add(1, Ordering::Relaxed);
                    }
                }
                if let Some(ref t) = self.cmd {
                    while let Some(cmd) = t.recv() {
                        if (cmd.linear - 2.0).abs() > 0.1 {
                            self.co.fetch_add(1, Ordering::Relaxed);
                        }
                        self.cr.fetch_add(1, Ordering::Relaxed);
                    }
                }
                if let Some(ref t) = self.bat {
                    while let Some(bat) = t.recv() {
                        if (bat.voltage - 12.6).abs() > 0.1 {
                            self.co.fetch_add(1, Ordering::Relaxed);
                        }
                        self.br.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        let mut sched = Scheduler::new().tick_rate(200_u64.hz());
        let _ = sched
            .add(MultiSub {
                imu: None,
                cmd: None,
                bat: None,
                ir,
                cr,
                br,
                co,
            })
            .rate(200_u64.hz())
            .order(0)
            .build();
        let _ = sched.run_for(Duration::from_secs(5));
    });

    std::thread::sleep(Duration::from_secs(1));

    // Start Python publisher
    let mut py = Command::new("python3")
        .arg(&py_file)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();

    let _ = py.wait();
    rust_handle.join().unwrap();

    let iv = imu_recv.load(Ordering::Relaxed);
    let cv = cmd_recv.load(Ordering::Relaxed);
    let bv = bat_recv.load(Ordering::Relaxed);
    let co = corrupt.load(Ordering::Relaxed);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  PYTHON→RUST: 3 MESSAGE TYPES                              ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!(
        "║  Imu:          recv {:5}  corrupt: {}                       ║",
        iv, co
    );
    println!(
        "║  CmdVel:       recv {:5}                                    ║",
        cv
    );
    println!(
        "║  BatteryState: recv {:5}                                    ║",
        bv
    );
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(co, 0, "DATA CORRUPTION in Python→Rust!");
    let types_ok = [iv > 0, cv > 0, bv > 0].iter().filter(|&&x| x).count();
    assert!(
        types_ok >= 2,
        "At least 2/3 types should deliver, got {}",
        types_ok
    );
    println!(
        "✓ rp_python_to_rust_3_types — {}/3 types, zero corruption",
        types_ok
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: Bidirectional — Rust scheduler + Python scheduler, both pub+sub
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn rp_bidirectional_rust_python_schedulers() {
    if !python_available() {
        println!("✓ SKIPPED (no horus python)");
        return;
    }
    cleanup_stale_shm();
    let _ = std::fs::remove_dir_all(result_dir());

    // Python: publishes CmdVel on "bidir.cmd", subscribes Imu on "bidir.imu"
    let py_code = r#"
import horus, os

imu_recv = [0]
cmd_sent = [0]

def tick(node):
    # Subscribe
    msg = node.recv("bidir.imu")
    while msg is not None:
        imu_recv[0] += 1
        msg = node.recv("bidir.imu")
    # Publish
    node.send("bidir.cmd", horus.CmdVel(0.5, 0.1))
    cmd_sent[0] += 1

def shutdown(node):
    d = os.environ.get("RESULT_DIR", "/tmp/horus_rp_matrix")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "py_bidir_imu_recv"), "w") as f: f.write(str(imu_recv[0]))
    with open(os.path.join(d, "py_bidir_cmd_sent"), "w") as f: f.write(str(cmd_sent[0]))

node = horus.Node("py_bidir", tick=tick,
    subs={"bidir.imu": horus.Imu},
    pubs={"bidir.cmd": horus.CmdVel},
    rate=100, shutdown=shutdown)
horus.run(node, duration=5.0, tick_rate=100)
"#;

    let tmpdir = tempfile::TempDir::new().unwrap();
    let py_file = tmpdir.path().join("bidir.py");
    std::fs::write(&py_file, py_code).unwrap();

    let imu_sent = Arc::new(AtomicU64::new(0));
    let cmd_recv = Arc::new(AtomicU64::new(0));
    let corrupt = Arc::new(AtomicU64::new(0));

    let is = imu_sent.clone();
    let cr = cmd_recv.clone();
    let co = corrupt.clone();

    // Rust: publishes Imu on "bidir.imu", subscribes CmdVel on "bidir.cmd"
    let rust_handle = std::thread::spawn(move || {
        struct BidirNode {
            imu_pub: Option<Topic<Imu>>,
            cmd_sub: Option<Topic<CmdVel>>,
            is: Arc<AtomicU64>,
            cr: Arc<AtomicU64>,
            co: Arc<AtomicU64>,
        }
        impl Node for BidirNode {
            fn name(&self) -> &str {
                "rust_bidir"
            }
            fn init(&mut self) -> horus_core::error::HorusResult<()> {
                self.imu_pub = Some(Topic::new("bidir.imu")?);
                self.cmd_sub = Some(Topic::new("bidir.cmd")?);
                Ok(())
            }
            fn tick(&mut self) {
                let mut imu = Imu::new();
                imu.linear_acceleration[2] = 9.81;
                if let Some(ref t) = self.imu_pub {
                    t.send(imu);
                }
                self.is.fetch_add(1, Ordering::Relaxed);
                if let Some(ref t) = self.cmd_sub {
                    while let Some(cmd) = t.recv() {
                        if !cmd.linear.is_finite() {
                            self.co.fetch_add(1, Ordering::Relaxed);
                        }
                        self.cr.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());
        let _ = sched
            .add(BidirNode {
                imu_pub: None,
                cmd_sub: None,
                is,
                cr,
                co,
            })
            .rate(100_u64.hz())
            .order(0)
            .build();
        let _ = sched.run_for(Duration::from_secs(4));
    });

    std::thread::sleep(Duration::from_secs(1));

    let mut py = Command::new("python3")
        .arg(&py_file)
        .env("RESULT_DIR", result_dir().to_str().unwrap())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();

    rust_handle.join().unwrap();
    let _ = py.wait();

    let rust_imu_sent = imu_sent.load(Ordering::Relaxed);
    let rust_cmd_recv = cmd_recv.load(Ordering::Relaxed);
    let rust_corrupt = corrupt.load(Ordering::Relaxed);
    let py_imu_recv: u64 = read_result("py_bidir_imu_recv").trim().parse().unwrap_or(0);
    let py_cmd_sent: u64 = read_result("py_bidir_cmd_sent").trim().parse().unwrap_or(0);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  BIDIRECTIONAL: Rust scheduler ↔ Python scheduler           ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!(
        "║  Rust→Python: Imu   sent {:5} → Python recv {:5}           ║",
        rust_imu_sent, py_imu_recv
    );
    println!(
        "║  Python→Rust: CmdVel sent {:5} → Rust recv {:5}            ║",
        py_cmd_sent, rust_cmd_recv
    );
    println!(
        "║  Corruption: {}                                              ║",
        rust_corrupt
    );
    println!("╚══════════════════════════════════════════════════════════════╝");

    assert_eq!(rust_corrupt, 0, "Corruption in bidirectional!");
    assert!(py_imu_recv > 0, "Python should receive Imu from Rust");
    assert!(rust_cmd_recv > 0, "Rust should receive CmdVel from Python");
    println!("✓ rp_bidirectional — both directions work, zero corruption");
}
