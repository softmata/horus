// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

//! Data Types Stress Test Suite
//!
//! Tests HORUS IPC with various data types and alignments:
//! - Primitives (u8-u64, i8-i64, f32, f64)
//! - Fixed-size arrays [T; N]
//! - Nested structs with different alignments
//! - C-style enums (#[repr(u8/u16/u32/u64)])
//! - Mixed alignment structs
//! - Cache-line aligned types
//!
//! Acceptance Criteria:
//! - All types serialize/deserialize correctly
//! - Alignment preserved in shared memory
//! - No padding corruption
//! - Nested structures work correctly

use bytemuck::{Pod, Zeroable};
use horus::prelude::Topic;
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use serde::{Serialize, Deserialize};
use serde_arrays;
use std::env;
use std::fmt;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

// ============================================================================
// PRIMITIVE MESSAGE TYPES
// ============================================================================

/// Message with all unsigned integer types
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct UnsignedIntegers {
    pub val_u8: u8,
    pub val_u16: u16,
    pub val_u32: u32,
    pub val_u64: u64,
}
unsafe impl Pod for UnsignedIntegers {}
unsafe impl Zeroable for UnsignedIntegers {}
unsafe impl PodMessage for UnsignedIntegers {}

/// Message with all signed integer types
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SignedIntegers {
    pub val_i8: i8,
    pub val_i16: i16,
    pub val_i32: i32,
    pub val_i64: i64,
}
unsafe impl Pod for SignedIntegers {}
unsafe impl Zeroable for SignedIntegers {}
unsafe impl PodMessage for SignedIntegers {}

/// Message with floating point types
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FloatingPoint {
    pub val_f32: f32,
    pub val_f64: f64,
}
unsafe impl Pod for FloatingPoint {}
unsafe impl Zeroable for FloatingPoint {}
unsafe impl PodMessage for FloatingPoint {}

/// Message with boolean (as u8)
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BooleanType {
    pub val_bool: u8,  // Booleans as u8 for Pod compatibility
    pub _pad: [u8; 7],
}
unsafe impl Pod for BooleanType {}
unsafe impl Zeroable for BooleanType {}
unsafe impl PodMessage for BooleanType {}

// ============================================================================
// ARRAY MESSAGE TYPES
// ============================================================================

/// Message with u8 array
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayU8 {
    pub len: u32,
    #[serde(with = "serde_arrays")]
    pub data: [u8; 60],
}
unsafe impl Pod for ArrayU8 {}
unsafe impl Zeroable for ArrayU8 {}
unsafe impl PodMessage for ArrayU8 {}

/// Message with u32 array
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayU32 {
    pub len: u32,
    #[serde(with = "serde_arrays")]
    pub data: [u32; 15],
}
unsafe impl Pod for ArrayU32 {}
unsafe impl Zeroable for ArrayU32 {}
unsafe impl PodMessage for ArrayU32 {}

/// Message with f64 array (common for robot joint data)
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayF64 {
    pub timestamp: u64,
    pub data: [f64; 8],  // 8 DOF robot
}
unsafe impl Pod for ArrayF64 {}
unsafe impl Zeroable for ArrayF64 {}
unsafe impl PodMessage for ArrayF64 {}

// ============================================================================
// NESTED STRUCT TYPES
// ============================================================================

/// Inner struct for nesting test
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
unsafe impl Pod for Vector3 {}
unsafe impl Zeroable for Vector3 {}

/// Inner struct for quaternion
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
unsafe impl Pod for Quaternion {}
unsafe impl Zeroable for Quaternion {}

/// Nested struct: Pose (position + orientation)
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Quaternion,
}
unsafe impl Pod for Pose {}
unsafe impl Zeroable for Pose {}
unsafe impl PodMessage for Pose {}

/// Deeply nested: Transform with frame IDs
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub timestamp: u64,
    pub frame_id: u32,
    pub child_frame_id: u32,
    pub pose: Pose,
}
unsafe impl Pod for Transform {}
unsafe impl Zeroable for Transform {}
unsafe impl PodMessage for Transform {}

impl fmt::Debug for Transform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Transform")
            .field("timestamp", &self.timestamp)
            .field("frame_id", &self.frame_id)
            .field("child_frame_id", &self.child_frame_id)
            .finish()
    }
}

// ============================================================================
// ENUM TYPES (C-style only)
// ============================================================================

/// C-style enum with u8 representation
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum StatusU8 {
    Idle = 0,
    Running = 1,
    Paused = 2,
    Error = 3,
    Complete = 4,
}
unsafe impl Pod for StatusU8 {}
unsafe impl Zeroable for StatusU8 {}

/// C-style enum with u32 representation
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum StatusU32 {
    Idle = 0,
    Running = 1,
    Paused = 2,
    Error = 0xFF00_0000,
    Complete = 0xFFFF_FFFF,
}
unsafe impl Pod for StatusU32 {}
unsafe impl Zeroable for StatusU32 {}

/// Message with enum field (u8)
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StatusMessageU8 {
    pub status: StatusU8,
    pub error_code: u8,
    pub _pad: [u8; 6],
}
unsafe impl Pod for StatusMessageU8 {}
unsafe impl Zeroable for StatusMessageU8 {}
unsafe impl PodMessage for StatusMessageU8 {}

/// Message with enum field (u32)
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StatusMessageU32 {
    pub status: StatusU32,
    pub error_code: u32,
}
unsafe impl Pod for StatusMessageU32 {}
unsafe impl Zeroable for StatusMessageU32 {}
unsafe impl PodMessage for StatusMessageU32 {}

// ============================================================================
// ALIGNMENT TEST TYPES
// ============================================================================

/// 1-byte aligned struct
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Align1 {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
}
unsafe impl Pod for Align1 {}
unsafe impl Zeroable for Align1 {}
unsafe impl PodMessage for Align1 {}

/// 2-byte aligned struct
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Align2 {
    pub a: u16,
    pub b: u16,
}
unsafe impl Pod for Align2 {}
unsafe impl Zeroable for Align2 {}
unsafe impl PodMessage for Align2 {}

/// 4-byte aligned struct
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Align4 {
    pub a: u32,
    pub b: u32,
}
unsafe impl Pod for Align4 {}
unsafe impl Zeroable for Align4 {}
unsafe impl PodMessage for Align4 {}

/// 8-byte aligned struct
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Align8 {
    pub a: u64,
    pub b: u64,
}
unsafe impl Pod for Align8 {}
unsafe impl Zeroable for Align8 {}
unsafe impl PodMessage for Align8 {}

/// Cache-line aligned (64 bytes)
#[repr(C, align(64))]
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CacheLineAligned {
    pub timestamp: u64,
    #[serde(with = "serde_arrays")]
    pub data: [u8; 56],
}
unsafe impl Pod for CacheLineAligned {}
unsafe impl Zeroable for CacheLineAligned {}
unsafe impl PodMessage for CacheLineAligned {}

impl fmt::Debug for CacheLineAligned {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CacheLineAligned")
            .field("timestamp", &self.timestamp)
            .field("data[0]", &self.data[0])
            .finish()
    }
}

// ============================================================================
// MIXED ALIGNMENT TYPES (realistic robotics data)
// ============================================================================

/// IMU data with mixed types
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ImuData {
    pub timestamp: u64,           // 8-byte aligned
    pub accel: [f32; 3],          // 4-byte aligned
    pub gyro: [f32; 3],           // 4-byte aligned
    pub mag: [f32; 3],            // 4-byte aligned
    pub temperature: f32,         // 4-byte aligned
}
unsafe impl Pod for ImuData {}
unsafe impl Zeroable for ImuData {}
unsafe impl PodMessage for ImuData {}

impl fmt::Debug for ImuData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ImuData")
            .field("timestamp", &self.timestamp)
            .field("accel", &self.accel)
            .field("gyro", &self.gyro)
            .finish()
    }
}

/// Joint state with mixed types
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JointState {
    pub timestamp: u64,
    pub num_joints: u32,
    pub _pad: u32,
    #[serde(with = "serde_arrays")]
    pub position: [f64; 7],   // 7 DOF arm
    #[serde(with = "serde_arrays")]
    pub velocity: [f64; 7],
    #[serde(with = "serde_arrays")]
    pub effort: [f64; 7],
}
unsafe impl Pod for JointState {}
unsafe impl Zeroable for JointState {}
unsafe impl PodMessage for JointState {}

impl fmt::Debug for JointState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JointState")
            .field("timestamp", &self.timestamp)
            .field("num_joints", &self.num_joints)
            .finish()
    }
}

/// Laser scan with fixed-size buffer
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LaserScan {
    pub timestamp: u64,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub num_ranges: u32,
    pub _pad: u32,
    #[serde(with = "serde_arrays")]
    pub ranges: [f32; 360],   // Fixed 360 point scan
}
unsafe impl Pod for LaserScan {}
unsafe impl Zeroable for LaserScan {}
unsafe impl PodMessage for LaserScan {}

impl fmt::Debug for LaserScan {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LaserScan")
            .field("timestamp", &self.timestamp)
            .field("num_ranges", &self.num_ranges)
            .finish()
    }
}

// ============================================================================
// LogSummary IMPLEMENTATIONS
// ============================================================================

impl LogSummary for UnsignedIntegers {
    fn log_summary(&self) -> String {
        format!("UnsignedIntegers(u8={},u16={},u32={},u64={})",
            self.val_u8, self.val_u16, self.val_u32, self.val_u64)
    }
}

impl LogSummary for SignedIntegers {
    fn log_summary(&self) -> String {
        format!("SignedIntegers(i8={},i16={},i32={},i64={})",
            self.val_i8, self.val_i16, self.val_i32, self.val_i64)
    }
}

impl LogSummary for FloatingPoint {
    fn log_summary(&self) -> String {
        format!("FloatingPoint(f32={:.3},f64={:.6})", self.val_f32, self.val_f64)
    }
}

impl LogSummary for BooleanType {
    fn log_summary(&self) -> String {
        format!("BooleanType(val={})", self.val_bool)
    }
}

impl LogSummary for ArrayU8 {
    fn log_summary(&self) -> String {
        format!("ArrayU8(len={})", self.len)
    }
}

impl LogSummary for ArrayU32 {
    fn log_summary(&self) -> String {
        format!("ArrayU32(len={})", self.len)
    }
}

impl LogSummary for ArrayF64 {
    fn log_summary(&self) -> String {
        format!("ArrayF64(ts={},data[0]={:.3})", self.timestamp, self.data[0])
    }
}

impl LogSummary for Vector3 {
    fn log_summary(&self) -> String {
        format!("Vector3({:.3},{:.3},{:.3})", self.x, self.y, self.z)
    }
}

impl LogSummary for Quaternion {
    fn log_summary(&self) -> String {
        format!("Quaternion(w={:.3},x={:.3},y={:.3},z={:.3})",
            self.w, self.x, self.y, self.z)
    }
}

impl LogSummary for Pose {
    fn log_summary(&self) -> String {
        format!("Pose(pos={},ori={})",
            self.position.log_summary(), self.orientation.log_summary())
    }
}

impl LogSummary for Transform {
    fn log_summary(&self) -> String {
        format!("Transform(ts={},frame={},child={})",
            self.timestamp, self.frame_id, self.child_frame_id)
    }
}

impl LogSummary for StatusU8 {
    fn log_summary(&self) -> String {
        format!("StatusU8({:?})", self)
    }
}

impl LogSummary for StatusU32 {
    fn log_summary(&self) -> String {
        format!("StatusU32({:?})", self)
    }
}

impl LogSummary for StatusMessageU8 {
    fn log_summary(&self) -> String {
        format!("StatusMessageU8(status={:?},err={})", self.status, self.error_code)
    }
}

impl LogSummary for StatusMessageU32 {
    fn log_summary(&self) -> String {
        format!("StatusMessageU32(status={:?},err={})", self.status, self.error_code)
    }
}

impl LogSummary for Align1 {
    fn log_summary(&self) -> String {
        format!("Align1(a={},b={},c={},d={})", self.a, self.b, self.c, self.d)
    }
}

impl LogSummary for Align2 {
    fn log_summary(&self) -> String {
        format!("Align2(a={},b={})", self.a, self.b)
    }
}

impl LogSummary for Align4 {
    fn log_summary(&self) -> String {
        format!("Align4(a={},b={})", self.a, self.b)
    }
}

impl LogSummary for Align8 {
    fn log_summary(&self) -> String {
        format!("Align8(a={},b={})", self.a, self.b)
    }
}

impl LogSummary for CacheLineAligned {
    fn log_summary(&self) -> String {
        format!("CacheLineAligned(ts={},data[0]={})", self.timestamp, self.data[0])
    }
}

impl LogSummary for ImuData {
    fn log_summary(&self) -> String {
        format!("ImuData(ts={},temp={:.1})", self.timestamp, self.temperature)
    }
}

impl LogSummary for JointState {
    fn log_summary(&self) -> String {
        format!("JointState(ts={},joints={})", self.timestamp, self.num_joints)
    }
}

impl LogSummary for LaserScan {
    fn log_summary(&self) -> String {
        format!("LaserScan(ts={},ranges={})", self.timestamp, self.num_ranges)
    }
}

// ============================================================================
// MAIN
// ============================================================================

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  primitives_unsigned   - Unsigned integer types");
        eprintln!("  primitives_signed     - Signed integer types");
        eprintln!("  primitives_float      - Floating point types");
        eprintln!("  arrays_u8             - u8 array transfer");
        eprintln!("  arrays_u32            - u32 array transfer");
        eprintln!("  arrays_f64            - f64 array transfer");
        eprintln!("  nested_pose           - Nested Pose struct");
        eprintln!("  nested_transform      - Deeply nested Transform");
        eprintln!("  enum_u8               - C-style enum (u8)");
        eprintln!("  enum_u32              - C-style enum (u32)");
        eprintln!("  align_1_2_4_8         - Alignment variations");
        eprintln!("  align_cache_line      - Cache-line alignment (64B)");
        eprintln!("  robotics_imu          - IMU data (mixed types)");
        eprintln!("  robotics_joint        - Joint state (7 DOF)");
        eprintln!("  robotics_laser        - Laser scan (360 points)");
        eprintln!("  all                   - Run all tests");
        process::exit(1);
    }

    let test_name = &args[1];

    let results = match test_name.as_str() {
        "primitives_unsigned" => vec![("primitives_unsigned", test_primitives_unsigned())],
        "primitives_signed" => vec![("primitives_signed", test_primitives_signed())],
        "primitives_float" => vec![("primitives_float", test_primitives_float())],
        "arrays_u8" => vec![("arrays_u8", test_arrays_u8())],
        "arrays_u32" => vec![("arrays_u32", test_arrays_u32())],
        "arrays_f64" => vec![("arrays_f64", test_arrays_f64())],
        "nested_pose" => vec![("nested_pose", test_nested_pose())],
        "nested_transform" => vec![("nested_transform", test_nested_transform())],
        "enum_u8" => vec![("enum_u8", test_enum_u8())],
        "enum_u32" => vec![("enum_u32", test_enum_u32())],
        "align_1_2_4_8" => vec![("align_1_2_4_8", test_alignment_variations())],
        "align_cache_line" => vec![("align_cache_line", test_cache_line_alignment())],
        "robotics_imu" => vec![("robotics_imu", test_robotics_imu())],
        "robotics_joint" => vec![("robotics_joint", test_robotics_joint())],
        "robotics_laser" => vec![("robotics_laser", test_robotics_laser())],
        "all" => run_all_tests(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    // Print summary
    println!("\n========================================");
    println!("Data Types Stress Test Results");
    println!("========================================");
    let mut all_passed = true;
    for (name, passed) in &results {
        let status = if *passed { "✓ PASS" } else { "✗ FAIL" };
        println!("  {}: {}", name, status);
        if !*passed {
            all_passed = false;
        }
    }
    println!("========================================");

    if all_passed {
        println!("All tests PASSED");
        process::exit(0);
    } else {
        eprintln!("Some tests FAILED");
        process::exit(1);
    }
}

fn run_all_tests() -> Vec<(&'static str, bool)> {
    vec![
        ("primitives_unsigned", test_primitives_unsigned()),
        ("primitives_signed", test_primitives_signed()),
        ("primitives_float", test_primitives_float()),
        ("arrays_u8", test_arrays_u8()),
        ("arrays_u32", test_arrays_u32()),
        ("arrays_f64", test_arrays_f64()),
        ("nested_pose", test_nested_pose()),
        ("nested_transform", test_nested_transform()),
        ("enum_u8", test_enum_u8()),
        ("enum_u32", test_enum_u32()),
        ("align_1_2_4_8", test_alignment_variations()),
        ("align_cache_line", test_cache_line_alignment()),
        ("robotics_imu", test_robotics_imu()),
        ("robotics_joint", test_robotics_joint()),
        ("robotics_laser", test_robotics_laser()),
    ]
}

// ============================================================================
// TEST IMPLEMENTATIONS
// ============================================================================

fn test_primitives_unsigned() -> bool {
    println!("\n========================================");
    println!("Test: Unsigned Integer Primitives");
    println!("========================================");

    let topic = format!("test_unsigned_{}", process::id());

    let pub_hub = match Topic::<UnsignedIntegers>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<UnsignedIntegers>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let test_values = [
        UnsignedIntegers { val_u8: 0, val_u16: 0, val_u32: 0, val_u64: 0 },
        UnsignedIntegers { val_u8: u8::MAX, val_u16: u16::MAX, val_u32: u32::MAX, val_u64: u64::MAX },
        UnsignedIntegers { val_u8: 127, val_u16: 32768, val_u32: 0x12345678, val_u64: 0xDEAD_BEEF_CAFE_BABE },
    ];

    let mut success = true;
    for (i, expected) in test_values.iter().enumerate() {
        if pub_hub.send(*expected, &mut None).is_err() {
            eprintln!("  Failed to send message {}", i);
            success = false;
            continue;
        }

        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received != *expected {
                eprintln!("  Message {} mismatch: {:?} != {:?}", i, received, expected);
                success = false;
            } else {
                println!("  Message {}: u8={}, u16={}, u32={:#x}, u64={:#x} ✓",
                    i, received.val_u8, received.val_u16, received.val_u32, received.val_u64);
            }
        } else {
            eprintln!("  Failed to receive message {}", i);
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<UnsignedIntegers>(),
        std::mem::align_of::<UnsignedIntegers>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_primitives_signed() -> bool {
    println!("\n========================================");
    println!("Test: Signed Integer Primitives");
    println!("========================================");

    let topic = format!("test_signed_{}", process::id());

    let pub_hub = match Topic::<SignedIntegers>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<SignedIntegers>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let test_values = [
        SignedIntegers { val_i8: 0, val_i16: 0, val_i32: 0, val_i64: 0 },
        SignedIntegers { val_i8: i8::MAX, val_i16: i16::MAX, val_i32: i32::MAX, val_i64: i64::MAX },
        SignedIntegers { val_i8: i8::MIN, val_i16: i16::MIN, val_i32: i32::MIN, val_i64: i64::MIN },
        SignedIntegers { val_i8: -1, val_i16: -1, val_i32: -1, val_i64: -1 },
    ];

    let mut success = true;
    for (i, expected) in test_values.iter().enumerate() {
        if pub_hub.send(*expected, &mut None).is_err() {
            eprintln!("  Failed to send message {}", i);
            success = false;
            continue;
        }

        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received != *expected {
                eprintln!("  Message {} mismatch: {:?} != {:?}", i, received, expected);
                success = false;
            } else {
                println!("  Message {}: i8={}, i16={}, i32={}, i64={} ✓",
                    i, received.val_i8, received.val_i16, received.val_i32, received.val_i64);
            }
        } else {
            eprintln!("  Failed to receive message {}", i);
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<SignedIntegers>(),
        std::mem::align_of::<SignedIntegers>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_primitives_float() -> bool {
    println!("\n========================================");
    println!("Test: Floating Point Primitives");
    println!("========================================");

    let topic = format!("test_float_{}", process::id());

    let pub_hub = match Topic::<FloatingPoint>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<FloatingPoint>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let test_values = [
        FloatingPoint { val_f32: 0.0, val_f64: 0.0 },
        FloatingPoint { val_f32: f32::MAX, val_f64: f64::MAX },
        FloatingPoint { val_f32: f32::MIN, val_f64: f64::MIN },
        FloatingPoint { val_f32: f32::MIN_POSITIVE, val_f64: f64::MIN_POSITIVE },
        FloatingPoint { val_f32: std::f32::consts::PI, val_f64: std::f64::consts::PI },
        FloatingPoint { val_f32: -1.5e10, val_f64: -1.5e100 },
    ];

    let mut success = true;
    for (i, expected) in test_values.iter().enumerate() {
        if pub_hub.send(*expected, &mut None).is_err() {
            eprintln!("  Failed to send message {}", i);
            success = false;
            continue;
        }

        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            // Use bitwise comparison for floating point
            if received.val_f32.to_bits() != expected.val_f32.to_bits() ||
               received.val_f64.to_bits() != expected.val_f64.to_bits() {
                eprintln!("  Message {} mismatch: {:?} != {:?}", i, received, expected);
                success = false;
            } else {
                println!("  Message {}: f32={:e}, f64={:e} ✓",
                    i, received.val_f32, received.val_f64);
            }
        } else {
            eprintln!("  Failed to receive message {}", i);
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<FloatingPoint>(),
        std::mem::align_of::<FloatingPoint>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_arrays_u8() -> bool {
    println!("\n========================================");
    println!("Test: u8 Array Transfer");
    println!("========================================");

    let topic = format!("test_array_u8_{}", process::id());

    let pub_hub = match Topic::<ArrayU8>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<ArrayU8>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    // Test various patterns
    let patterns: Vec<u8> = vec![0x00, 0xFF, 0xAA, 0x55, 0x12];
    let mut success = true;

    for (i, &pattern) in patterns.iter().enumerate() {
        let mut data = [0u8; 60];
        for (j, byte) in data.iter_mut().enumerate() {
            *byte = pattern.wrapping_add(j as u8);
        }

        let msg = ArrayU8 { len: 60, data };

        if pub_hub.send(msg, &mut None).is_err() {
            eprintln!("  Failed to send message {}", i);
            success = false;
            continue;
        }

        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received.data != msg.data || received.len != msg.len {
                eprintln!("  Message {} array mismatch", i);
                success = false;
            } else {
                println!("  Pattern {:#04x}: len={}, data[0]={:#04x}, data[59]={:#04x} ✓",
                    pattern, received.len, received.data[0], received.data[59]);
            }
        } else {
            eprintln!("  Failed to receive message {}", i);
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<ArrayU8>(),
        std::mem::align_of::<ArrayU8>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_arrays_u32() -> bool {
    println!("\n========================================");
    println!("Test: u32 Array Transfer");
    println!("========================================");

    let topic = format!("test_array_u32_{}", process::id());

    let pub_hub = match Topic::<ArrayU32>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<ArrayU32>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut data = [0u32; 15];
    for (i, val) in data.iter_mut().enumerate() {
        *val = 0xDEADBEEF_u32.wrapping_add(i as u32 * 0x1111_1111);
    }

    let msg = ArrayU32 { len: 15, data };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received.data != msg.data || received.len != msg.len {
                eprintln!("  Array mismatch");
                success = false;
            } else {
                println!("  u32 array: len={}, data[0]={:#010x}, data[14]={:#010x} ✓",
                    received.len, received.data[0], received.data[14]);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<ArrayU32>(),
        std::mem::align_of::<ArrayU32>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_arrays_f64() -> bool {
    println!("\n========================================");
    println!("Test: f64 Array Transfer (Robot Joints)");
    println!("========================================");

    let topic = format!("test_array_f64_{}", process::id());

    let pub_hub = match Topic::<ArrayF64>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<ArrayF64>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    // Typical robot joint positions in radians
    let data = [
        0.0,                          // joint 0
        std::f64::consts::PI / 4.0,   // joint 1
        -std::f64::consts::PI / 2.0,  // joint 2
        std::f64::consts::PI,         // joint 3
        -std::f64::consts::PI,        // joint 4
        0.5,                          // joint 5
        -0.5,                         // joint 6
        1.234567890123456789,         // joint 7 (precision test)
    ];

    let msg = ArrayF64 { timestamp: 1234567890, data };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            let mut match_ok = received.timestamp == msg.timestamp;
            for i in 0..8 {
                if received.data[i].to_bits() != msg.data[i].to_bits() {
                    match_ok = false;
                    break;
                }
            }

            if !match_ok {
                eprintln!("  Array mismatch");
                success = false;
            } else {
                println!("  f64 array: timestamp={}, joint[0]={:.6}, joint[7]={:.15} ✓",
                    received.timestamp, received.data[0], received.data[7]);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<ArrayF64>(),
        std::mem::align_of::<ArrayF64>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_nested_pose() -> bool {
    println!("\n========================================");
    println!("Test: Nested Struct (Pose)");
    println!("========================================");

    let topic = format!("test_nested_pose_{}", process::id());

    let pub_hub = match Topic::<Pose>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Pose>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let msg = Pose {
        position: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
        orientation: Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
    };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received != msg {
                eprintln!("  Nested struct mismatch");
                success = false;
            } else {
                println!("  Pose: position=({:.1}, {:.1}, {:.1}), orientation=({:.1}, {:.1}, {:.1}, {:.1}) ✓",
                    received.position.x, received.position.y, received.position.z,
                    received.orientation.w, received.orientation.x, received.orientation.y, received.orientation.z);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes (Vector3={}, Quaternion={}), Align: {} bytes",
        std::mem::size_of::<Pose>(),
        std::mem::size_of::<Vector3>(),
        std::mem::size_of::<Quaternion>(),
        std::mem::align_of::<Pose>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_nested_transform() -> bool {
    println!("\n========================================");
    println!("Test: Deeply Nested Struct (Transform)");
    println!("========================================");

    let topic = format!("test_nested_transform_{}", process::id());

    let pub_hub = match Topic::<Transform>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Transform>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let msg = Transform {
        timestamp: 1234567890123456789,
        frame_id: 1,
        child_frame_id: 2,
        pose: Pose {
            position: Vector3 { x: 10.5, y: -20.3, z: 100.7 },
            orientation: Quaternion {
                w: 0.707,
                x: 0.0,
                y: 0.707,
                z: 0.0
            },
        },
    };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received != msg {
                eprintln!("  Deeply nested struct mismatch");
                success = false;
            } else {
                println!("  Transform: ts={}, frame={}->{}, pos=({:.1}, {:.1}, {:.1}) ✓",
                    received.timestamp, received.frame_id, received.child_frame_id,
                    received.pose.position.x, received.pose.position.y, received.pose.position.z);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes (nesting: Transform > Pose > Vector3+Quaternion), Align: {} bytes",
        std::mem::size_of::<Transform>(),
        std::mem::align_of::<Transform>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_enum_u8() -> bool {
    println!("\n========================================");
    println!("Test: C-style Enum (u8)");
    println!("========================================");

    let topic = format!("test_enum_u8_{}", process::id());

    let pub_hub = match Topic::<StatusMessageU8>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<StatusMessageU8>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let test_values = [
        StatusMessageU8 { status: StatusU8::Idle, error_code: 0, _pad: [0; 6] },
        StatusMessageU8 { status: StatusU8::Running, error_code: 0, _pad: [0; 6] },
        StatusMessageU8 { status: StatusU8::Paused, error_code: 1, _pad: [0; 6] },
        StatusMessageU8 { status: StatusU8::Error, error_code: 255, _pad: [0; 6] },
        StatusMessageU8 { status: StatusU8::Complete, error_code: 0, _pad: [0; 6] },
    ];

    let mut success = true;
    for (i, expected) in test_values.iter().enumerate() {
        if pub_hub.send(*expected, &mut None).is_err() {
            eprintln!("  Failed to send message {}", i);
            success = false;
            continue;
        }

        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received.status != expected.status || received.error_code != expected.error_code {
                eprintln!("  Message {} mismatch: {:?} != {:?}", i, received, expected);
                success = false;
            } else {
                println!("  Message {}: status={:?}, error_code={} ✓",
                    i, received.status, received.error_code);
            }
        } else {
            eprintln!("  Failed to receive message {}", i);
            success = false;
        }
    }

    println!("  Enum size: {} byte, Message size: {} bytes",
        std::mem::size_of::<StatusU8>(),
        std::mem::size_of::<StatusMessageU8>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_enum_u32() -> bool {
    println!("\n========================================");
    println!("Test: C-style Enum (u32)");
    println!("========================================");

    let topic = format!("test_enum_u32_{}", process::id());

    let pub_hub = match Topic::<StatusMessageU32>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<StatusMessageU32>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let test_values = [
        StatusMessageU32 { status: StatusU32::Idle, error_code: 0 },
        StatusMessageU32 { status: StatusU32::Running, error_code: 0x12345678 },
        StatusMessageU32 { status: StatusU32::Error, error_code: 0xDEADBEEF },
        StatusMessageU32 { status: StatusU32::Complete, error_code: u32::MAX },
    ];

    let mut success = true;
    for (i, expected) in test_values.iter().enumerate() {
        if pub_hub.send(*expected, &mut None).is_err() {
            eprintln!("  Failed to send message {}", i);
            success = false;
            continue;
        }

        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received.status != expected.status || received.error_code != expected.error_code {
                eprintln!("  Message {} mismatch: {:?} != {:?}", i, received, expected);
                success = false;
            } else {
                println!("  Message {}: status={:?}, error_code={:#010x} ✓",
                    i, received.status, received.error_code);
            }
        } else {
            eprintln!("  Failed to receive message {}", i);
            success = false;
        }
    }

    println!("  Enum size: {} bytes, Message size: {} bytes",
        std::mem::size_of::<StatusU32>(),
        std::mem::size_of::<StatusMessageU32>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_alignment_variations() -> bool {
    println!("\n========================================");
    println!("Test: Alignment Variations (1, 2, 4, 8 bytes)");
    println!("========================================");

    let mut success = true;

    // Test 1-byte alignment
    {
        let topic = format!("test_align1_{}", process::id());
        let pub_hub = Topic::<Align1>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Align1>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        let msg = Align1 { a: 0x11, b: 0x22, c: 0x33, d: 0x44 };
        pub_hub.send(msg, &mut None).expect("send");
        thread::sleep(Duration::from_millis(10));

        if let Some(recv) = sub_hub.recv(&mut None) {
            if recv == msg {
                println!("  Align1 (packed): size={}, align={} ✓",
                    std::mem::size_of::<Align1>(), std::mem::align_of::<Align1>());
            } else {
                eprintln!("  Align1 mismatch");
                success = false;
            }
        } else {
            eprintln!("  Align1 recv failed");
            success = false;
        }
    }

    // Test 2-byte alignment
    {
        let topic = format!("test_align2_{}", process::id());
        let pub_hub = Topic::<Align2>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Align2>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        let msg = Align2 { a: 0x1122, b: 0x3344 };
        pub_hub.send(msg, &mut None).expect("send");
        thread::sleep(Duration::from_millis(10));

        if let Some(recv) = sub_hub.recv(&mut None) {
            if recv == msg {
                println!("  Align2: size={}, align={} ✓",
                    std::mem::size_of::<Align2>(), std::mem::align_of::<Align2>());
            } else {
                eprintln!("  Align2 mismatch");
                success = false;
            }
        } else {
            eprintln!("  Align2 recv failed");
            success = false;
        }
    }

    // Test 4-byte alignment
    {
        let topic = format!("test_align4_{}", process::id());
        let pub_hub = Topic::<Align4>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Align4>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        let msg = Align4 { a: 0x11223344, b: 0x55667788 };
        pub_hub.send(msg, &mut None).expect("send");
        thread::sleep(Duration::from_millis(10));

        if let Some(recv) = sub_hub.recv(&mut None) {
            if recv == msg {
                println!("  Align4: size={}, align={} ✓",
                    std::mem::size_of::<Align4>(), std::mem::align_of::<Align4>());
            } else {
                eprintln!("  Align4 mismatch");
                success = false;
            }
        } else {
            eprintln!("  Align4 recv failed");
            success = false;
        }
    }

    // Test 8-byte alignment
    {
        let topic = format!("test_align8_{}", process::id());
        let pub_hub = Topic::<Align8>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Align8>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        let msg = Align8 { a: 0x1122334455667788, b: 0x99AABBCCDDEEFF00 };
        pub_hub.send(msg, &mut None).expect("send");
        thread::sleep(Duration::from_millis(10));

        if let Some(recv) = sub_hub.recv(&mut None) {
            if recv == msg {
                println!("  Align8: size={}, align={} ✓",
                    std::mem::size_of::<Align8>(), std::mem::align_of::<Align8>());
            } else {
                eprintln!("  Align8 mismatch");
                success = false;
            }
        } else {
            eprintln!("  Align8 recv failed");
            success = false;
        }
    }

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_cache_line_alignment() -> bool {
    println!("\n========================================");
    println!("Test: Cache-Line Alignment (64 bytes)");
    println!("========================================");

    let topic = format!("test_cache_line_{}", process::id());

    let pub_hub = match Topic::<CacheLineAligned>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<CacheLineAligned>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut data = [0u8; 56];
    for (i, byte) in data.iter_mut().enumerate() {
        *byte = (i * 7) as u8;  // Some pattern
    }

    let msg = CacheLineAligned { timestamp: 0xCAFE_BABE_DEAD_BEEF, data };

    let mut success = true;

    // Verify alignment at compile time
    let align = std::mem::align_of::<CacheLineAligned>();
    if align != 64 {
        eprintln!("  Expected alignment 64, got {}", align);
        success = false;
    }

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            if received.timestamp != msg.timestamp || received.data != msg.data {
                eprintln!("  Cache-line aligned message mismatch");
                success = false;
            } else {
                println!("  CacheLineAligned: timestamp={:#018x}, data[0]={}, data[55]={} ✓",
                    received.timestamp, received.data[0], received.data[55]);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes (cache line)",
        std::mem::size_of::<CacheLineAligned>(),
        std::mem::align_of::<CacheLineAligned>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_robotics_imu() -> bool {
    println!("\n========================================");
    println!("Test: Robotics IMU Data (Mixed Types)");
    println!("========================================");

    let topic = format!("test_imu_{}", process::id());

    let pub_hub = match Topic::<ImuData>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<ImuData>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let msg = ImuData {
        timestamp: 1234567890123456789,
        accel: [0.1, -9.81, 0.05],      // m/s^2
        gyro: [0.001, -0.002, 0.003],   // rad/s
        mag: [25.0, -30.0, 50.0],       // μT
        temperature: 25.5,              // °C
    };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            // Verify each field
            let mut match_ok = received.timestamp == msg.timestamp;
            match_ok &= received.temperature.to_bits() == msg.temperature.to_bits();
            for i in 0..3 {
                match_ok &= received.accel[i].to_bits() == msg.accel[i].to_bits();
                match_ok &= received.gyro[i].to_bits() == msg.gyro[i].to_bits();
                match_ok &= received.mag[i].to_bits() == msg.mag[i].to_bits();
            }

            if !match_ok {
                eprintln!("  IMU data mismatch");
                success = false;
            } else {
                println!("  IMU: accel=({:.2}, {:.2}, {:.2}) m/s², gyro=({:.4}, {:.4}, {:.4}) rad/s ✓",
                    received.accel[0], received.accel[1], received.accel[2],
                    received.gyro[0], received.gyro[1], received.gyro[2]);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes, Align: {} bytes",
        std::mem::size_of::<ImuData>(),
        std::mem::align_of::<ImuData>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_robotics_joint() -> bool {
    println!("\n========================================");
    println!("Test: Robotics Joint State (7 DOF)");
    println!("========================================");

    let topic = format!("test_joint_{}", process::id());

    let pub_hub = match Topic::<JointState>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<JointState>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let msg = JointState {
        timestamp: 1234567890123456789,
        num_joints: 7,
        _pad: 0,
        position: [0.0, 0.5, -1.0, 1.5, -0.5, 0.25, 0.0],
        velocity: [0.1, -0.1, 0.2, -0.2, 0.0, 0.0, 0.0],
        effort: [10.0, 20.0, 30.0, 40.0, 5.0, 5.0, 2.0],
    };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            // Verify each field
            let mut match_ok = received.timestamp == msg.timestamp;
            match_ok &= received.num_joints == msg.num_joints;
            for i in 0..7 {
                match_ok &= received.position[i].to_bits() == msg.position[i].to_bits();
                match_ok &= received.velocity[i].to_bits() == msg.velocity[i].to_bits();
                match_ok &= received.effort[i].to_bits() == msg.effort[i].to_bits();
            }

            if !match_ok {
                eprintln!("  Joint state mismatch");
                success = false;
            } else {
                println!("  JointState: joints={}, pos[0]={:.2}, vel[0]={:.2}, eff[0]={:.1} ✓",
                    received.num_joints, received.position[0], received.velocity[0], received.effort[0]);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes (7 joints × 3 arrays × 8 bytes = {} + header), Align: {} bytes",
        std::mem::size_of::<JointState>(),
        7 * 3 * 8,
        std::mem::align_of::<JointState>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

fn test_robotics_laser() -> bool {
    println!("\n========================================");
    println!("Test: Robotics Laser Scan (360 points)");
    println!("========================================");

    let topic = format!("test_laser_{}", process::id());

    let pub_hub = match Topic::<LaserScan>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<LaserScan>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    // Generate realistic laser scan data
    let mut ranges = [0.0f32; 360];
    for (i, range) in ranges.iter_mut().enumerate() {
        // Simulate a rectangular room
        let angle = (i as f64) * std::f64::consts::PI / 180.0;
        *range = (5.0 / angle.cos().abs().max(0.1)) as f32;
        *range = range.min(30.0);  // Max range
    }

    let msg = LaserScan {
        timestamp: 1234567890123456789,
        angle_min: 0.0,
        angle_max: 2.0 * std::f32::consts::PI,
        angle_increment: std::f32::consts::PI / 180.0,
        range_min: 0.1,
        range_max: 30.0,
        num_ranges: 360,
        _pad: 0,
        ranges,
    };

    let mut success = true;

    if pub_hub.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send message");
        success = false;
    } else {
        thread::sleep(Duration::from_millis(10));

        if let Some(received) = sub_hub.recv(&mut None) {
            // Verify key fields and some range samples
            let mut match_ok = received.timestamp == msg.timestamp;
            match_ok &= received.num_ranges == msg.num_ranges;
            match_ok &= received.angle_min.to_bits() == msg.angle_min.to_bits();
            match_ok &= received.range_max.to_bits() == msg.range_max.to_bits();

            // Check some sample ranges
            for i in [0, 90, 180, 270, 359] {
                match_ok &= received.ranges[i].to_bits() == msg.ranges[i].to_bits();
            }

            if !match_ok {
                eprintln!("  Laser scan mismatch");
                success = false;
            } else {
                println!("  LaserScan: {} points, range[0]={:.2}m, range[90]={:.2}m, range[180]={:.2}m ✓",
                    received.num_ranges, received.ranges[0], received.ranges[90], received.ranges[180]);
            }
        } else {
            eprintln!("  Failed to receive message");
            success = false;
        }
    }

    println!("  Size: {} bytes (360 × 4 = {} bytes for ranges), Align: {} bytes",
        std::mem::size_of::<LaserScan>(),
        360 * 4,
        std::mem::align_of::<LaserScan>());

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}
