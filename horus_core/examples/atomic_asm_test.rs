//! Atomic operations assembly test
//!
//! This file contains isolated atomic operations to verify optimal codegen.
//! Build with: RUSTFLAGS="-C target-cpu=native" cargo build --release --example atomic_asm_test
//! View assembly with: cargo asm --lib -p horus_core --example atomic_asm_test <function_name>

use std::sync::atomic::{AtomicU64, Ordering};

/// Test atomic load with Acquire ordering
/// On x86-64: Should compile to plain MOV (no fence needed)
#[inline(never)]
#[no_mangle]
pub fn atomic_load_acquire(counter: &AtomicU64) -> u64 {
    counter.load(Ordering::Acquire)
}

/// Test atomic load with Relaxed ordering
/// On x86-64: Should compile to plain MOV
#[inline(never)]
#[no_mangle]
pub fn atomic_load_relaxed(counter: &AtomicU64) -> u64 {
    counter.load(Ordering::Relaxed)
}

/// Test atomic store with Release ordering
/// On x86-64: Should compile to plain MOV (no fence needed)
#[inline(never)]
#[no_mangle]
pub fn atomic_store_release(counter: &AtomicU64, value: u64) {
    counter.store(value, Ordering::Release)
}

/// Test atomic store with Relaxed ordering
/// On x86-64: Should compile to plain MOV
#[inline(never)]
#[no_mangle]
pub fn atomic_store_relaxed(counter: &AtomicU64, value: u64) {
    counter.store(value, Ordering::Relaxed)
}

/// Test fetch_add with Release ordering
/// On x86-64: MUST use LOCK XADD (atomic read-modify-write)
#[inline(never)]
#[no_mangle]
pub fn atomic_fetch_add_release(counter: &AtomicU64, value: u64) -> u64 {
    counter.fetch_add(value, Ordering::Release)
}

/// Test fetch_add with Relaxed ordering
/// On x86-64: MUST use LOCK XADD (atomic read-modify-write)
#[inline(never)]
#[no_mangle]
pub fn atomic_fetch_add_relaxed(counter: &AtomicU64, value: u64) -> u64 {
    counter.fetch_add(value, Ordering::Relaxed)
}

/// Test compare_exchange
/// On x86-64: MUST use LOCK CMPXCHG
#[inline(never)]
#[no_mangle]
pub fn atomic_compare_exchange(counter: &AtomicU64, expected: u64, new: u64) -> Result<u64, u64> {
    counter.compare_exchange(expected, new, Ordering::Release, Ordering::Relaxed)
}

fn main() {
    let counter = AtomicU64::new(0);

    // Use all functions to prevent dead code elimination
    println!("Load Acquire: {}", atomic_load_acquire(&counter));
    println!("Load Relaxed: {}", atomic_load_relaxed(&counter));

    atomic_store_release(&counter, 42);
    atomic_store_relaxed(&counter, 100);

    println!("Fetch Add Release: {}", atomic_fetch_add_release(&counter, 1));
    println!("Fetch Add Relaxed: {}", atomic_fetch_add_relaxed(&counter, 1));

    match atomic_compare_exchange(&counter, 102, 200) {
        Ok(v) => println!("CAS succeeded, old: {}", v),
        Err(v) => println!("CAS failed, current: {}", v),
    }

    println!("Final value: {}", counter.load(Ordering::Relaxed));
}
