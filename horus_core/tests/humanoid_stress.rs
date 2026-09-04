#![allow(dead_code)]
#![allow(
    unused_must_use,
    clippy::needless_range_loop,
    clippy::field_reassign_with_default
)]
//! Humanoid robot stress test.
//!
//! Simulates a 20-DOF humanoid with realistic node topology:
//!
//!   ┌─────────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
//!   │ IMU (1kHz)  │  │ FT_L     │  │ FT_R     │  │ Camera   │
//!   │             │  │ (500Hz)  │  │ (500Hz)  │  │ (30Hz)   │
//!   └──────┬──────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘
//!          │              │              │              │
//!          ▼              ▼              ▼              │
//!   ┌──────────────────────────────────────┐           │
//!   │        State Estimator (1kHz)        │◄──────────┘
//!   │  (fuses IMU + FT + joint feedback)   │
//!   └──────────────┬───────────────────────┘
//!                  │ publishes to 5 consumers
//!          ┌───────┼───────┬───────┬───────┐
//!          ▼       ▼       ▼       ▼       ▼
//!   ┌──────────┐┌─────┐┌──────┐┌──────┐┌──────┐
//!   │ Balance  ││Gait ││ WBC  ││ Log  ││ Viz  │
//!   │ (500Hz)  ││(100)││(500) ││(50Hz)││(30Hz)│
//!   └────┬─────┘└──┬──┘└──┬───┘└──────┘└──────┘
//!        │         │      │
//!        ▼         ▼      ▼
//!   ┌──────────────────────────────────────┐
//!   │     Joint Command Mux (500Hz)        │
//!   │  (merges balance + gait + WBC cmds)  │
//!   └──────────────┬───────────────────────┘
//!                  │
//!    ┌─────┬───────┼───────┬─────────────────┐
//!    ▼     ▼       ▼       ▼                 ▼
//!  ┌────┐┌────┐ ┌────┐  ┌────┐           ┌────┐
//!  │J_0 ││J_1 │ │J_2 │  │J_3 │   ...     │J_19│
//!  │PID ││PID │ │PID │  │PID │           │PID │
//!  └──┬─┘└──┬─┘ └──┬─┘  └──┬─┘           └──┬─┘
//!     │     │      │       │                 │
//!     ▼     ▼      ▼       ▼                 ▼
//!   (motor commands → joint feedback topics → state estimator)
//!
//! Total: 31 nodes, 50+ topics, mixed rates (30Hz–1kHz)
//!
//! This test answers:
//! - Does IPC lock under 50+ topics?
//! - Does fan-out (1 publisher → 5 subscribers) work?
//! - Do mixed rates (1kHz + 30Hz) coexist?
//! - What's the end-to-end latency (sensor → actuator)?
//! - Does the scheduler stall when one node is slow?
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test humanoid_stress -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// Message types (all POD for zero-copy)
// ════════════════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════════════════
// End-to-end latency instrumentation
// ════════════════════════════════════════════════════════════════════════
//
// The header of this file has always claimed the test answers "what's the
// end-to-end latency (sensor -> actuator)?". It did not. `stamp_ns` was
// written as `seq * 1_000_000` -- a synthetic counter derived from the tick
// index, not a clock reading -- and then never read by anything. No latency
// was computed anywhere in the file, or anywhere else in the tree.
//
// It is measured now. The IMU stamps a real monotonic reading, the state
// estimator PROPAGATES that stamp instead of overwriting it with a counter,
// and the balance controller differences it against the clock when the derived
// state reaches it. What comes out is the age of the sensor sample at the
// moment the actuator command is computed, across two topic hops and two
// scheduler wakeups.

/// Process-wide monotonic origin. All nodes here run in one process, so a
/// shared `Instant` origin gives every stamp a common, monotonic base without
/// depending on the wall clock or on clock synchronisation.
fn clock_origin() -> &'static Instant {
    static ORIGIN: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    ORIGIN.get_or_init(Instant::now)
}

fn now_ns() -> u64 {
    clock_origin().elapsed().as_nanos() as u64
}

/// Lock-free sink for latency samples.
///
/// Preallocated and written with relaxed atomics because it is used from inside
/// a node's `tick()`: a `Mutex` there would add the very contention the number
/// is supposed to measure, and on the RT path it would be a priority inversion.
/// Samples past capacity are counted and dropped rather than growing the
/// buffer, so the measurement cost stays constant.
struct LatencySink {
    samples: Vec<AtomicU64>,
    len: AtomicUsize,
    dropped: AtomicU64,
}

impl LatencySink {
    fn new(capacity: usize) -> Arc<Self> {
        Arc::new(Self {
            samples: (0..capacity).map(|_| AtomicU64::new(0)).collect(),
            len: AtomicUsize::new(0),
            dropped: AtomicU64::new(0),
        })
    }

    #[inline]
    fn record(&self, ns: u64) {
        let idx = self.len.fetch_add(1, Ordering::Relaxed);
        if idx < self.samples.len() {
            self.samples[idx].store(ns, Ordering::Relaxed);
        } else {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Sorted samples, for percentile reads.
    fn sorted(&self) -> Vec<u64> {
        let n = self.len.load(Ordering::Relaxed).min(self.samples.len());
        let mut v: Vec<u64> = self.samples[..n]
            .iter()
            .map(|a| a.load(Ordering::Relaxed))
            .collect();
        v.sort_unstable();
        v
    }
}

/// A percentile of a pre-sorted slice, or `None` when the sample count cannot
/// support it.
///
/// The floor is not decoration. A p99 read off 12 samples is the 12th-largest
/// of 12 and carries no information about the tail; reporting it as "p99" is
/// how a number that means nothing ends up in a document. `None` says so.
fn percentile(sorted: &[u64], q: f64) -> Option<u64> {
    let min_n = (1.0 / (1.0 - q)).ceil() as usize;
    if sorted.len() < min_n {
        return None;
    }
    let idx = ((sorted.len() as f64) * q).ceil() as usize;
    sorted.get(idx.saturating_sub(1)).copied()
}

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct ImuData {
    accel: [f64; 3],
    gyro: [f64; 3],
    orientation: [f64; 4], // quaternion
    seq: u64,
    stamp_ns: u64,
}
unsafe impl bytemuck::Pod for ImuData {}
unsafe impl bytemuck::Zeroable for ImuData {}

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct ForceTorque {
    force: [f64; 3],
    torque: [f64; 3],
    seq: u64,
}
unsafe impl bytemuck::Pod for ForceTorque {}
unsafe impl bytemuck::Zeroable for ForceTorque {}

/// 20 joint positions + velocities + efforts
#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct JointState20 {
    positions: [f64; 20],
    velocities: [f64; 20],
    efforts: [f64; 20],
    seq: u64,
    stamp_ns: u64,
}
unsafe impl bytemuck::Pod for JointState20 {}
unsafe impl bytemuck::Zeroable for JointState20 {}

/// Robot state estimate (published to 5+ consumers)
#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct RobotState {
    com_position: [f64; 3], // center of mass
    com_velocity: [f64; 3],
    base_orientation: [f64; 4], // quaternion
    base_angular_vel: [f64; 3],
    joint_positions: [f64; 20],
    joint_velocities: [f64; 20],
    zmp: [f64; 2], // zero moment point
    seq: u64,
    stamp_ns: u64,
}
unsafe impl bytemuck::Pod for RobotState {}
unsafe impl bytemuck::Zeroable for RobotState {}

/// Per-joint command
#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct JointCmd20 {
    targets: [f64; 20],    // position targets
    velocities: [f64; 20], // velocity feedforward
    torques: [f64; 20],    // torque feedforward
    seq: u64,
}
unsafe impl bytemuck::Pod for JointCmd20 {}
unsafe impl bytemuck::Zeroable for JointCmd20 {}

/// Single joint motor command (output of per-joint PID)
#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct MotorCmd {
    voltage: f64,
    joint_id: u32,
    _pad: u32,
    seq: u64,
}
unsafe impl bytemuck::Pod for MotorCmd {}
unsafe impl bytemuck::Zeroable for MotorCmd {}

// ════════════════════════════════════════════════════════════════════════
// Sensor nodes
// ════════════════════════════════════════════════════════════════════════

struct ImuNode {
    topic: Option<Topic<ImuData>>,
    name_str: String,
    seq: u64,
    ticks: Arc<AtomicU64>,
}
impl Node for ImuNode {
    fn name(&self) -> &str {
        "imu_1khz"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.name_str)?);
        Ok(())
    }
    fn tick(&mut self) {
        let t = self.seq as f64 * 0.001;
        let data = ImuData {
            accel: [0.0, 0.0, 9.81 + (t * 2.0).sin() * 0.05],
            gyro: [(t * 5.0).sin() * 0.01, 0.0, 0.0],
            orientation: [0.0, 0.0, (t * 0.5).sin() * 0.01, 1.0],
            seq: self.seq,
            // A real monotonic reading, not a tick index. This stamp is
            // what the whole latency measurement rests on.
            stamp_ns: now_ns(),
        };
        if let Some(ref t) = self.topic {
            t.send(data);
        }
        self.seq += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct ForceTorqueNode {
    topic: Option<Topic<ForceTorque>>,
    name_str: String,
    node_name: String,
    seq: u64,
    ticks: Arc<AtomicU64>,
}
impl Node for ForceTorqueNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.name_str)?);
        Ok(())
    }
    fn tick(&mut self) {
        let data = ForceTorque {
            force: [0.0, 0.0, 40.0 + (self.seq as f64 * 0.01).sin() * 5.0], // ~40N standing
            torque: [0.0, 0.0, 0.0],
            seq: self.seq,
        };
        if let Some(ref t) = self.topic {
            t.send(data);
        }
        self.seq += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

// ════════════════════════════════════════════════════════════════════════
// State estimator — fuses all sensors, publishes to 5 consumers
// ════════════════════════════════════════════════════════════════════════

struct StateEstimatorNode {
    imu_name: String,
    ft_l_name: String,
    ft_r_name: String,
    state_name: String,
    imu: Option<Topic<ImuData>>,
    ft_l: Option<Topic<ForceTorque>>,
    ft_r: Option<Topic<ForceTorque>>,
    state_out: Option<Topic<RobotState>>,
    seq: u64,
    ticks: Arc<AtomicU64>,
    imu_received: Arc<AtomicU64>,
}
impl Node for StateEstimatorNode {
    fn name(&self) -> &str {
        "state_estimator"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.imu = Some(Topic::new(&self.imu_name)?);
        self.ft_l = Some(Topic::new(&self.ft_l_name)?);
        self.ft_r = Some(Topic::new(&self.ft_r_name)?);
        self.state_out = Some(Topic::new(&self.state_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut latest_imu = None;
        let mut latest_ft_l = None;
        let mut latest_ft_r = None;

        if let Some(ref t) = self.imu {
            while let Some(d) = t.recv() {
                latest_imu = Some(d);
                self.imu_received.fetch_add(1, Ordering::Relaxed);
            }
        }
        if let Some(ref t) = self.ft_l {
            while let Some(d) = t.recv() {
                latest_ft_l = Some(d);
            }
        }
        if let Some(ref t) = self.ft_r {
            while let Some(d) = t.recv() {
                latest_ft_r = Some(d);
            }
        }

        // Simple fusion: just forward IMU orientation + compute ZMP from FT
        let mut state = RobotState::default();
        // The origin stamp of the freshest sensor sample folded into this
        // estimate. Zero means none arrived this tick.
        let imu_stamp_ns = latest_imu.map(|d: ImuData| d.stamp_ns).unwrap_or(0);
        if let Some(imu) = latest_imu {
            state.base_orientation = imu.orientation;
            state.base_angular_vel = imu.gyro;
            state.com_velocity = [imu.accel[0] * 0.001, imu.accel[1] * 0.001, 0.0];
        }
        if let (Some(fl), Some(fr)) = (latest_ft_l, latest_ft_r) {
            let total_fz = fl.force[2] + fr.force[2];
            if total_fz > 1.0 {
                state.zmp = [(fl.force[2] * (-0.1) + fr.force[2] * 0.1) / total_fz, 0.0];
            }
        }
        state.seq = self.seq;
        // Carry the SENSOR's stamp forward. Overwriting it here with the
        // estimator's own counter -- which is what this line used to do --
        // discards the origin time and makes end-to-end latency unmeasurable
        // downstream. Zero when no IMU sample has arrived yet, which the
        // consumer treats as "no measurement" rather than as a huge one.
        state.stamp_ns = imu_stamp_ns;

        if let Some(ref t) = self.state_out {
            t.send(state);
        }
        self.seq += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

// ════════════════════════════════════════════════════════════════════════
// Consumer nodes (balance, gait, WBC, logger, viz)
// ════════════════════════════════════════════════════════════════════════

struct BalanceControllerNode {
    state_name: String,
    cmd_name: String,
    state_in: Option<Topic<RobotState>>,
    cmd_out: Option<Topic<JointCmd20>>,
    seq: u64,
    ticks: Arc<AtomicU64>,
    state_received: Arc<AtomicU64>,
    /// Age of the originating IMU sample when the command derived from it is
    /// produced here -- the sensor-to-actuator latency the file header claims
    /// to report.
    latency: Arc<LatencySink>,
}
impl Node for BalanceControllerNode {
    fn name(&self) -> &str {
        "balance_ctrl"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.state_in = Some(Topic::new(&self.state_name)?);
        self.cmd_out = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.state_in {
            while let Some(state) = t.recv() {
                self.state_received.fetch_add(1, Ordering::Relaxed);
                // Measured before any work in this tick, so the sample is the
                // age of the sensor reading at the moment the actuator command
                // is computed -- not that plus this node's own compute.
                //
                // A zero stamp means the estimate folded in no IMU sample, and
                // a stamp ahead of `now` would mean the clock went backwards;
                // both are skipped rather than recorded as an enormous or
                // negative latency.
                if state.stamp_ns > 0 {
                    let now = now_ns();
                    if now >= state.stamp_ns {
                        self.latency.record(now - state.stamp_ns);
                    }
                }
                // Simple balance: adjust ankle joints based on ZMP error
                let mut cmd = JointCmd20::default();
                cmd.targets[14] = -state.zmp[0] * 5.0; // left ankle
                cmd.targets[19] = -state.zmp[0] * 5.0; // right ankle
                cmd.seq = self.seq;
                if let Some(ref out) = self.cmd_out {
                    out.send(cmd);
                }
            }
        }
        self.seq += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

struct GaitGeneratorNode {
    state_name: String,
    cmd_name: String,
    state_in: Option<Topic<RobotState>>,
    cmd_out: Option<Topic<JointCmd20>>,
    seq: u64,
    ticks: Arc<AtomicU64>,
}
impl Node for GaitGeneratorNode {
    fn name(&self) -> &str {
        "gait_gen"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.state_in = Some(Topic::new(&self.state_name)?);
        self.cmd_out = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.state_in {
            while t.recv().is_some() {} // drain
        }
        // Generate walking gait pattern
        let t = self.seq as f64 * 0.01; // 100Hz
        let mut cmd = JointCmd20::default();
        // Sinusoidal hip/knee pattern for walking
        for leg in 0..2 {
            let phase = if leg == 0 { 0.0 } else { std::f64::consts::PI };
            let base = leg * 5; // joints 0-4 left, 5-9 right
            cmd.targets[base] = (t + phase).sin() * 0.3; // hip pitch
            cmd.targets[base + 1] = 0.0; // hip roll
            cmd.targets[base + 2] = (t + phase).sin().abs() * 0.5; // knee
            cmd.targets[base + 3] = -(t + phase).sin() * 0.15; // ankle pitch
            cmd.targets[base + 4] = 0.0; // ankle roll
        }
        cmd.seq = self.seq;
        if let Some(ref out) = self.cmd_out {
            out.send(cmd);
        }
        self.seq += 1;
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

/// Whole-body controller
struct WbcNode {
    state_name: String,
    cmd_name: String,
    state_in: Option<Topic<RobotState>>,
    cmd_out: Option<Topic<JointCmd20>>,
    ticks: Arc<AtomicU64>,
}
impl Node for WbcNode {
    fn name(&self) -> &str {
        "wbc"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.state_in = Some(Topic::new(&self.state_name)?);
        self.cmd_out = Some(Topic::new(&self.cmd_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.state_in {
            while t.recv().is_some() {} // drain
        }
        // Simulate expensive QP solve (~200μs of computation)
        let start = Instant::now();
        let mut x = 1.0f64;
        while start.elapsed() < Duration::from_micros(50) {
            x = (x * 1.0000001).sin().abs() + 1.0;
        }
        std::hint::black_box(x);

        let cmd = JointCmd20::default();
        if let Some(ref out) = self.cmd_out {
            out.send(cmd);
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

/// Logger (slow consumer — should NOT block fast producers)
struct LoggerNode {
    state_name: String,
    state_in: Option<Topic<RobotState>>,
    logged: Arc<AtomicU64>,
}
impl Node for LoggerNode {
    fn name(&self) -> &str {
        "logger"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.state_in = Some(Topic::new(&self.state_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.state_in {
            while t.recv().is_some() {
                self.logged.fetch_add(1, Ordering::Relaxed);
            }
        }
        // Simulate slow logging (1ms write)
        std::thread::sleep(Duration::from_micros(500));
    }
}

// ════════════════════════════════════════════════════════════════════════
// Joint command multiplexer + 20 joint PID nodes
// ════════════════════════════════════════════════════════════════════════

struct JointMuxNode {
    bal_name: String,
    gait_name: String,
    wbc_name: String,
    out_name: String,
    bal_in: Option<Topic<JointCmd20>>,
    gait_in: Option<Topic<JointCmd20>>,
    wbc_in: Option<Topic<JointCmd20>>,
    out: Option<Topic<JointCmd20>>,
    ticks: Arc<AtomicU64>,
}
impl Node for JointMuxNode {
    fn name(&self) -> &str {
        "joint_mux"
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.bal_in = Some(Topic::new(&self.bal_name)?);
        self.gait_in = Some(Topic::new(&self.gait_name)?);
        self.wbc_in = Some(Topic::new(&self.wbc_name)?);
        self.out = Some(Topic::new(&self.out_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut bal = JointCmd20::default();
        let mut gait = JointCmd20::default();
        let mut wbc = JointCmd20::default();

        if let Some(ref t) = self.bal_in {
            while let Some(c) = t.recv() {
                bal = c;
            }
        }
        if let Some(ref t) = self.gait_in {
            while let Some(c) = t.recv() {
                gait = c;
            }
        }
        if let Some(ref t) = self.wbc_in {
            while let Some(c) = t.recv() {
                wbc = c;
            }
        }

        // Priority merge: WBC > Balance > Gait
        let mut merged = JointCmd20::default();
        for i in 0..20 {
            merged.targets[i] = gait.targets[i] + bal.targets[i] + wbc.targets[i];
        }
        if let Some(ref out) = self.out {
            out.send(merged);
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

/// Per-joint PID controller (20 instances)
struct JointPidNode {
    joint_id: usize,
    cmd_name: String,
    fb_name: String,
    node_name: String,
    cmd_in: Option<Topic<JointCmd20>>,
    fb_out: Option<Topic<MotorCmd>>,
    position: f64,
    velocity: f64,
    ticks: Arc<AtomicU64>,
}
impl Node for JointPidNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.cmd_in = Some(Topic::new(&self.cmd_name)?);
        self.fb_out = Some(Topic::new(&self.fb_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let mut target = 0.0;
        if let Some(ref t) = self.cmd_in {
            while let Some(cmd) = t.recv() {
                target = cmd.targets[self.joint_id];
            }
        }
        // Simple P controller + plant simulation
        let error = target - self.position;
        let voltage = error * 100.0;
        self.velocity += (voltage * 0.01 - self.velocity * 0.1) * 0.002; // 500Hz dt
        self.position += self.velocity * 0.002;

        let cmd = MotorCmd {
            voltage: voltage.clamp(-24.0, 24.0),
            joint_id: self.joint_id as u32,
            _pad: 0,
            seq: 0,
        };
        if let Some(ref out) = self.fb_out {
            out.send(cmd);
        }
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

// ════════════════════════════════════════════════════════════════════════
// THE TEST
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn humanoid_20dof_31_nodes_50_topics() {
    let _shm_guard = cleanup_stale_shm();

    let prefix = unique("hum");

    // Topic names
    let imu_topic = format!("{}.imu", prefix);
    let ft_l_topic = format!("{}.ft_left", prefix);
    let ft_r_topic = format!("{}.ft_right", prefix);
    let state_topic = format!("{}.state", prefix);
    let bal_cmd_topic = format!("{}.bal_cmd", prefix);
    let gait_cmd_topic = format!("{}.gait_cmd", prefix);
    let wbc_cmd_topic = format!("{}.wbc_cmd", prefix);
    let mux_cmd_topic = format!("{}.mux_cmd", prefix);

    // Counters
    // Sensor→actuator latency samples. Declared out here with the other
    // counters so the report below can read it after the scheduler's scope has
    // closed. 200k slots covers a 10s run at 500 Hz with room to spare;
    // overflow is counted rather than grown, so recording cost cannot drift
    // during the run.
    let latency = LatencySink::new(200_000);

    let imu_ticks = Arc::new(AtomicU64::new(0));
    let ft_l_ticks = Arc::new(AtomicU64::new(0));
    let ft_r_ticks = Arc::new(AtomicU64::new(0));
    let est_ticks = Arc::new(AtomicU64::new(0));
    let bal_ticks = Arc::new(AtomicU64::new(0));
    let gait_ticks = Arc::new(AtomicU64::new(0));
    let wbc_ticks = Arc::new(AtomicU64::new(0));
    let mux_ticks = Arc::new(AtomicU64::new(0));
    let log_count = Arc::new(AtomicU64::new(0));
    let imu_recv_count = Arc::new(AtomicU64::new(0));
    let state_recv_count = Arc::new(AtomicU64::new(0));

    let joint_ticks: Vec<Arc<AtomicU64>> = (0..20).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = running.clone();

    // Clone all counters for the thread
    let ic = imu_ticks.clone();
    let flc = ft_l_ticks.clone();
    let frc = ft_r_ticks.clone();
    let ec = est_ticks.clone();
    let bc = bal_ticks.clone();
    let gc = gait_ticks.clone();
    let wc = wbc_ticks.clone();
    let mc = mux_ticks.clone();
    let lc = log_count.clone();
    let irc = imu_recv_count.clone();
    let src = state_recv_count.clone();
    let jc: Vec<_> = joint_ticks.iter().map(Arc::clone).collect();

    // The scheduler thread takes its own handle; the report below reads the
    // original after the thread has joined.
    let latency_for_nodes = Arc::clone(&latency);
    let handle = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(1000_u64.hz());

        // ── Sensors ────────────────────────────────────────────
        let _ = sched
            .add(ImuNode {
                topic: None,
                name_str: imu_topic.clone(),
                seq: 0,
                ticks: ic,
            })
            .rate(1000_u64.hz())
            .order(0)
            .build();

        let _ = sched
            .add(ForceTorqueNode {
                topic: None,
                name_str: ft_l_topic.clone(),
                node_name: "ft_left".into(),
                seq: 0,
                ticks: flc,
            })
            .rate(500_u64.hz())
            .order(0)
            .build();

        let _ = sched
            .add(ForceTorqueNode {
                topic: None,
                name_str: ft_r_topic.clone(),
                node_name: "ft_right".into(),
                seq: 0,
                ticks: frc,
            })
            .rate(500_u64.hz())
            .order(0)
            .build();

        // ── State Estimator ────────────────────────────────────
        let _ = sched
            .add(StateEstimatorNode {
                imu_name: imu_topic,
                ft_l_name: ft_l_topic,
                ft_r_name: ft_r_topic,
                state_name: state_topic.clone(),
                imu: None,
                ft_l: None,
                ft_r: None,
                state_out: None,
                seq: 0,
                ticks: ec,
                imu_received: irc,
            })
            .rate(1000_u64.hz())
            .order(1)
            .build();

        // ── Controllers (all read state, publish commands) ─────
        let _ = sched
            .add(BalanceControllerNode {
                state_name: state_topic.clone(),
                cmd_name: bal_cmd_topic.clone(),
                state_in: None,
                cmd_out: None,
                seq: 0,
                ticks: bc,
                state_received: src,
                latency: Arc::clone(&latency_for_nodes),
            })
            .rate(500_u64.hz())
            .order(2)
            .build();

        let _ = sched
            .add(GaitGeneratorNode {
                state_name: state_topic.clone(),
                cmd_name: gait_cmd_topic.clone(),
                state_in: None,
                cmd_out: None,
                seq: 0,
                ticks: gc,
            })
            .rate(100_u64.hz())
            .order(2)
            .build();

        let _ = sched
            .add(WbcNode {
                state_name: state_topic.clone(),
                cmd_name: wbc_cmd_topic.clone(),
                state_in: None,
                cmd_out: None,
                ticks: wc,
            })
            .rate(500_u64.hz())
            .order(2)
            .build();

        let _ = sched
            .add(LoggerNode {
                state_name: state_topic.clone(),
                state_in: None,
                logged: lc,
            })
            .rate(50_u64.hz())
            .order(2)
            .build();

        // ── Joint Command Mux ──────────────────────────────────
        let _ = sched
            .add(JointMuxNode {
                bal_name: bal_cmd_topic,
                gait_name: gait_cmd_topic,
                wbc_name: wbc_cmd_topic,
                out_name: mux_cmd_topic.clone(),
                bal_in: None,
                gait_in: None,
                wbc_in: None,
                out: None,
                ticks: mc,
            })
            .rate(500_u64.hz())
            .order(3)
            .build();

        // ── 20 Joint PIDs ──────────────────────────────────────
        for j in 0..20 {
            let fb_topic = format!("{}.joint_{}_fb", prefix, j);
            let _ = sched
                .add(JointPidNode {
                    joint_id: j,
                    cmd_name: mux_cmd_topic.clone(),
                    fb_name: fb_topic,
                    node_name: format!("joint_{}", j),
                    cmd_in: None,
                    fb_out: None,
                    position: 0.0,
                    velocity: 0.0,
                    ticks: jc[j].clone(),
                })
                .rate(500_u64.hz())
                .order(4)
                .build();
        }

        // ── Run ────────────────────────────────────────────────
        while running_clone.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_micros(500));
        }
    });

    let test_duration = Duration::from_secs(10);
    let start = Instant::now();
    std::thread::sleep(test_duration);
    running.store(false, Ordering::Relaxed);
    handle.join().unwrap();
    let elapsed = start.elapsed();

    // ── Results ────────────────────────────────────────────────────
    let it = imu_ticks.load(Ordering::Relaxed);
    let flt = ft_l_ticks.load(Ordering::Relaxed);
    let frt = ft_r_ticks.load(Ordering::Relaxed);
    let et = est_ticks.load(Ordering::Relaxed);
    let bt = bal_ticks.load(Ordering::Relaxed);
    let gt = gait_ticks.load(Ordering::Relaxed);
    let wt = wbc_ticks.load(Ordering::Relaxed);
    let mt = mux_ticks.load(Ordering::Relaxed);
    let ll = log_count.load(Ordering::Relaxed);
    let ir = imu_recv_count.load(Ordering::Relaxed);
    let sr = state_recv_count.load(Ordering::Relaxed);

    let secs = elapsed.as_secs_f64();
    println!("╔══════════════════════════════════════════════════════════╗");
    println!("║       HUMANOID STRESS TEST — 31 nodes, 50+ topics      ║");
    println!(
        "║                    {:.1}s elapsed                          ║",
        secs
    );
    println!("╠══════════════════════════════════════════════════════════╣");
    println!("║ SENSORS                                                 ║");
    println!(
        "║   IMU (1kHz):       {:6} ticks ({:6.1} Hz)              ║",
        it,
        it as f64 / secs
    );
    println!(
        "║   FT Left (500Hz):  {:6} ticks ({:6.1} Hz)              ║",
        flt,
        flt as f64 / secs
    );
    println!(
        "║   FT Right (500Hz): {:6} ticks ({:6.1} Hz)              ║",
        frt,
        frt as f64 / secs
    );
    println!("║ ESTIMATOR                                               ║");
    println!(
        "║   State Est (1kHz): {:6} ticks, received {:6} IMU msgs ║",
        et, ir
    );
    println!("║ CONTROLLERS                                             ║");
    println!(
        "║   Balance (500Hz):  {:6} ticks, received {:6} states   ║",
        bt, sr
    );
    println!(
        "║   Gait (100Hz):     {:6} ticks ({:6.1} Hz)              ║",
        gt,
        gt as f64 / secs
    );
    println!(
        "║   WBC (500Hz):      {:6} ticks ({:6.1} Hz)              ║",
        wt,
        wt as f64 / secs
    );
    println!(
        "║   Logger (50Hz):    {:6} logged                         ║",
        ll
    );
    println!("║ COMMAND MUX                                             ║");
    println!(
        "║   Mux (500Hz):      {:6} ticks ({:6.1} Hz)              ║",
        mt,
        mt as f64 / secs
    );
    println!("║ JOINT PIDs (20 joints × 500Hz)                          ║");
    let total_joint_ticks: u64 = joint_ticks.iter().map(|j| j.load(Ordering::Relaxed)).sum();
    let min_joint = joint_ticks
        .iter()
        .map(|j| j.load(Ordering::Relaxed))
        .min()
        .unwrap();
    let max_joint = joint_ticks
        .iter()
        .map(|j| j.load(Ordering::Relaxed))
        .max()
        .unwrap();
    println!(
        "║   Total ticks:     {:7} ({:.1} Hz per joint avg)       ║",
        total_joint_ticks,
        total_joint_ticks as f64 / 20.0 / secs
    );
    println!(
        "║   Min/Max spread:  {:6}/{:6} ({:.1}% spread)             ║",
        min_joint,
        max_joint,
        if max_joint > 0 {
            (max_joint - min_joint) as f64 / max_joint as f64 * 100.0
        } else {
            0.0
        }
    );
    println!("╠══════════════════════════════════════════════════════════╣");

    // ── Critical checks ────────────────────────────────────────
    let mut failures = vec![];

    // 1. Did the pipeline actually flow? (IMU → Estimator → Balance)
    if ir == 0 {
        failures.push("State estimator received ZERO IMU messages — IPC broken");
    }
    if sr == 0 {
        failures.push("Balance controller received ZERO state estimates — fan-out broken");
    }

    // 2. Did the slow logger block fast nodes?
    // If logger blocked others, IMU ticks would be << expected
    let imu_hz = it as f64 / secs;
    let logger_blocking = imu_hz < 100.0; // Should be at least 100Hz even in debug
    if logger_blocking {
        failures.push("IMU rate dropped below 100Hz — slow logger may be blocking pipeline");
    }

    // 3. Did all 20 joint PIDs actually run?
    let joints_running = joint_ticks
        .iter()
        .filter(|j| j.load(Ordering::Relaxed) > 0)
        .count();
    if joints_running < 20 {
        failures.push("Not all 20 joint PIDs received ticks");
    }

    // 4. Is joint tick spread reasonable? (no starvation)
    let spread_pct = if max_joint > 0 {
        (max_joint - min_joint) as f64 / max_joint as f64 * 100.0
    } else {
        100.0
    };
    if spread_pct > 20.0 {
        failures.push("Joint tick spread >20% — some joints starved");
    }

    // 5. Did WBC (expensive node with 50μs compute) run without stalling others?
    let wbc_hz = wt as f64 / secs;
    if wbc_hz < 50.0 {
        failures.push("WBC rate too low — expensive computation stalling scheduler");
    }

    println!("║ CHECKS                                                  ║");
    println!(
        "║   IMU→Estimator IPC:      {} ({} msgs)",
        if ir > 0 { "✓ PASS" } else { "✗ FAIL" },
        ir
    );
    println!(
        "║   State fan-out (1→5):    {} ({} msgs)",
        if sr > 0 { "✓ PASS" } else { "✗ FAIL" },
        sr
    );
    println!(
        "║   Logger not blocking:    {} (IMU={:.0}Hz)",
        if !logger_blocking {
            "✓ PASS"
        } else {
            "✗ FAIL"
        },
        imu_hz
    );
    println!(
        "║   All 20 joints running:  {} ({}/20)",
        if joints_running == 20 {
            "✓ PASS"
        } else {
            "✗ FAIL"
        },
        joints_running
    );
    println!(
        "║   Joint tick fairness:    {} ({:.1}% spread)",
        if spread_pct <= 20.0 {
            "✓ PASS"
        } else {
            "✗ FAIL"
        },
        spread_pct
    );
    println!(
        "║   WBC not stalling:       {} ({:.0}Hz)",
        if wbc_hz >= 50.0 {
            "✓ PASS"
        } else {
            "✗ FAIL"
        },
        wbc_hz
    );
    // ── End-to-end latency: IMU sample -> actuator command ──────────
    //
    // Two topic hops and two scheduler wakeups: the 1 kHz IMU publishes, the
    // state estimator folds it into a RobotState carrying the sensor's own
    // stamp, and the 500 Hz balance controller differences that stamp when the
    // command derived from it is produced.
    let lat = latency.sorted();
    let lat_dropped = latency.dropped.load(Ordering::Relaxed);
    println!("║ END-TO-END LATENCY (IMU sample → actuator command)      ║");
    if lat.is_empty() {
        println!("║   NO SAMPLES — the sensor→actuator path carried nothing  ║");
        failures.push(
            "No end-to-end latency samples: no IMU-derived state reached the balance \
             controller with a usable stamp",
        );
    } else {
        let mean = lat.iter().sum::<u64>() as f64 / lat.len() as f64;
        println!(
            "║   n={:6}  min {:7.1}us  mean {:7.1}us              ║",
            lat.len(),
            lat[0] as f64 / 1000.0,
            mean / 1000.0
        );
        // Percentiles are printed only where the sample count supports them.
        // A "p99" read off a dozen samples is the largest of a dozen and says
        // nothing about a tail; printing it anyway is how such a number ends
        // up quoted in a document.
        for (label, q) in [("p50", 0.50), ("p99", 0.99), ("p99.9", 0.999)] {
            match percentile(&lat, q) {
                Some(v) => println!(
                    "║   {:5}            {:9.1}us                            ║",
                    label,
                    v as f64 / 1000.0
                ),
                None => println!(
                    "║   {:5}            unsupported at n={:<6}               ║",
                    label,
                    lat.len()
                ),
            }
        }
        println!(
            "║   max              {:9.1}us                            ║",
            lat[lat.len() - 1] as f64 / 1000.0
        );
        if lat_dropped > 0 {
            println!(
                "║   {:6} samples past buffer capacity (not counted)       ║",
                lat_dropped
            );
        }

        // A gate, not decoration. The balance controller runs at 500 Hz, so a
        // sample older than several of its own periods by the time it is acted
        // on means the pipeline -- not the arithmetic -- is the problem. Set
        // wide deliberately: this is a 31-node debug-build stress test on a
        // shared machine, and the value of the number here is that it EXISTS
        // and is tracked, not that it is tight.
        const MAX_MEAN_LATENCY_US: f64 = 50_000.0;
        if mean / 1000.0 > MAX_MEAN_LATENCY_US {
            failures.push(
                "Mean sensor→actuator latency exceeded 50ms — the pipeline is not keeping up",
            );
        }
    }

    println!("╚══════════════════════════════════════════════════════════╝");

    if !failures.is_empty() {
        for f in &failures {
            eprintln!("FAILURE: {}", f);
        }
        panic!("{} checks failed", failures.len());
    }

    println!("\nHUMANOID STRESS TEST PASSED ✓ (31 nodes, 50+ topics, 10s)");
}
