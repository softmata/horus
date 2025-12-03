//! Integration tests for CUDA IPC tensor sharing
//!
//! These tests verify that GPU tensors can be shared across processes
//! using CUDA IPC handles with zero-copy semantics.
//!
//! Run with: cargo test -p horus_core --test cuda_ipc_integration --features cuda

#![cfg(feature = "cuda")]

use std::io::Write;
use std::process::{Command, Stdio};
use std::time::Instant;

/// Test that CUDA is available and working
#[test]
fn test_cuda_availability() {
    use horus_core::memory::{cuda_available, cuda_device_count};

    if !cuda_available() {
        eprintln!("Skipping CUDA tests - no GPU available");
        return;
    }

    let count = cuda_device_count();
    assert!(count > 0, "CUDA available but no devices found");
    println!("CUDA available with {} device(s)", count);
}

/// Test CudaTensorPool creation and allocation
#[test]
fn test_cuda_tensor_pool_alloc() {
    use horus_core::memory::tensor_pool::TensorDtype;
    use horus_core::memory::{cuda_available, CudaTensorPool, CudaTensorPoolConfig};

    if !cuda_available() {
        eprintln!("Skipping - CUDA not available");
        return;
    }

    // Create pool
    let pool = CudaTensorPool::new(100, 0, CudaTensorPoolConfig::default())
        .expect("Failed to create CUDA pool");

    assert_eq!(pool.pool_id(), 100);
    assert_eq!(pool.device_id(), 0);
    assert!(pool.is_owner());

    // Allocate tensor
    let tensor = pool
        .alloc(&[1024, 1024], TensorDtype::F32)
        .expect("Failed to allocate tensor");

    assert_eq!(tensor.pool_id, 100);
    assert_eq!(tensor.device_id, 0);
    assert_eq!(tensor.numel, 1024 * 1024);
    assert_eq!(tensor.size, 1024 * 1024 * 4); // f32 = 4 bytes

    // IPC handle should be populated
    let handle = tensor.ipc_handle_bytes();
    assert_eq!(handle.len(), 64);
    assert!(
        handle.iter().any(|&b| b != 0),
        "IPC handle should not be all zeros"
    );

    // Check stats
    let stats = pool.stats();
    assert_eq!(stats.allocated_slots, 1);

    // Release
    pool.release(&tensor).expect("Failed to release tensor");

    let stats = pool.stats();
    assert_eq!(stats.allocated_slots, 0);

    println!("CUDA tensor pool test passed!");
}

/// Test IPC handle generation and metadata
/// Note: CUDA IPC cannot be opened in the same process that created it,
/// so we only test handle generation here. Cross-process test is separate.
#[test]
fn test_cuda_ipc_handle_generation() {
    use horus_core::memory::tensor_pool::TensorDtype;
    use horus_core::memory::{cuda_available, CudaTensorPool, CudaTensorPoolConfig};

    if !cuda_available() {
        eprintln!("Skipping - CUDA not available");
        return;
    }

    // Create pool as "owner"
    let pool = CudaTensorPool::new(101, 0, CudaTensorPoolConfig::default())
        .expect("Failed to create CUDA pool");

    // Allocate tensor
    let tensor = pool
        .alloc(&[256, 256, 3], TensorDtype::U8)
        .expect("Failed to allocate tensor");

    let ipc_handle = tensor.ipc_handle_bytes();

    // IPC handle should be 64 bytes
    assert_eq!(ipc_handle.len(), 64);

    // Handle should not be all zeros (that would indicate failure)
    assert!(
        ipc_handle.iter().any(|&b| b != 0),
        "IPC handle should not be all zeros"
    );

    // Tensor metadata should be correct
    assert_eq!(tensor.numel, 256 * 256 * 3);
    assert_eq!(tensor.size, 256 * 256 * 3); // u8 = 1 byte
    assert_eq!(tensor.ndim, 3);

    // Get device pointer (owner can use this directly)
    let gpu_ptr = pool.device_ptr(&tensor);
    assert!(!gpu_ptr.is_null(), "GPU pointer should not be null");

    // Release tensor
    pool.release(&tensor).expect("Failed to release tensor");

    println!("CUDA IPC handle generation test passed!");
}

/// Test multiple tensor allocations
#[test]
fn test_cuda_multiple_tensors() {
    use horus_core::memory::tensor_pool::TensorDtype;
    use horus_core::memory::{cuda_available, CudaTensorPool, CudaTensorPoolConfig};

    if !cuda_available() {
        eprintln!("Skipping - CUDA not available");
        return;
    }

    let pool = CudaTensorPool::new(102, 0, CudaTensorPoolConfig::default())
        .expect("Failed to create CUDA pool");

    // Allocate multiple tensors
    let tensors: Vec<_> = (0..10)
        .map(|i| {
            pool.alloc(&[100 + i as u64, 100], TensorDtype::F32)
                .expect("Failed to allocate tensor")
        })
        .collect();

    let stats = pool.stats();
    assert_eq!(stats.allocated_slots, 10);

    // Release all
    for tensor in &tensors {
        pool.release(tensor).expect("Failed to release");
    }

    let stats = pool.stats();
    assert_eq!(stats.allocated_slots, 0);

    println!("CUDA multiple tensors test passed!");
}

/// Test tensor allocation performance
#[test]
fn test_cuda_alloc_performance() {
    use horus_core::memory::tensor_pool::TensorDtype;
    use horus_core::memory::{cuda_available, CudaTensorPool, CudaTensorPoolConfig};

    if !cuda_available() {
        eprintln!("Skipping - CUDA not available");
        return;
    }

    let pool = CudaTensorPool::new(103, 0, CudaTensorPoolConfig::default())
        .expect("Failed to create CUDA pool");

    let iterations = 100;

    // Measure allocation time
    let start = Instant::now();
    let tensors: Vec<_> = (0..iterations)
        .map(|_| pool.alloc(&[1920, 1080, 3], TensorDtype::F32).unwrap())
        .collect();
    let alloc_time = start.elapsed();

    // Measure release time
    let start = Instant::now();
    for tensor in &tensors {
        pool.release(tensor).unwrap();
    }
    let release_time = start.elapsed();

    let alloc_per_tensor = alloc_time.as_micros() as f64 / iterations as f64;
    let release_per_tensor = release_time.as_micros() as f64 / iterations as f64;

    println!("CUDA allocation performance:");
    println!("  Alloc time per 1080p tensor: {:.2}μs", alloc_per_tensor);
    println!("  Release time per tensor: {:.2}μs", release_per_tensor);

    // Reasonable performance expectations
    assert!(
        alloc_per_tensor < 1000.0,
        "Allocation too slow: {}μs",
        alloc_per_tensor
    );
}

/// Test IPC handle size
#[test]
fn test_ipc_handle_size() {
    use horus_core::memory::CUDA_IPC_HANDLE_SIZE;

    // CUDA IPC handles are always 64 bytes
    assert_eq!(CUDA_IPC_HANDLE_SIZE, 64);
}

/// Real cross-process test using child process
/// This test spawns a child process that shares GPU memory with the parent
#[test]
#[ignore] // Run with: cargo test -p horus_core --test cuda_ipc_integration --features cuda -- --ignored
fn test_cuda_ipc_cross_process() {
    use horus_core::memory::cuda_ffi::{device_synchronize, memcpy, CudaMemcpyKind};
    use horus_core::memory::tensor_pool::TensorDtype;
    use horus_core::memory::{cuda_available, CudaTensorPool, CudaTensorPoolConfig};

    if !cuda_available() {
        eprintln!("Skipping - CUDA not available");
        return;
    }

    // Check if we're the child process
    if std::env::var("HORUS_IPC_TEST_CHILD").is_ok() {
        // Child process: read IPC handle from stdin and verify
        let mut handle_hex = String::new();
        std::io::stdin().read_line(&mut handle_hex).unwrap();
        let handle: Vec<u8> = (0..handle_hex.trim().len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&handle_hex.trim()[i..i + 2], 16).unwrap())
            .collect();

        // Open pool and import handle
        let pool = CudaTensorPool::open(200, 0).expect("Child: Failed to open pool");

        let (gpu_ptr, _tensor) = pool
            .import_ipc(&handle, &[4], TensorDtype::U32)
            .expect("Child: Failed to import IPC handle");

        // Read values from GPU
        let mut values = [0u32; 4];
        memcpy(
            values.as_mut_ptr() as *mut std::ffi::c_void,
            gpu_ptr as *const std::ffi::c_void,
            16,
            CudaMemcpyKind::DeviceToHost,
        )
        .expect("Child: memcpy failed");

        // Verify magic values
        assert_eq!(values, [0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x87654321]);
        println!("Child: Verified GPU memory contents!");

        pool.close_ipc(gpu_ptr).unwrap();
        return;
    }

    // Parent process
    println!("Parent: Creating CUDA pool and tensor...");

    let pool = CudaTensorPool::new(200, 0, CudaTensorPoolConfig::default())
        .expect("Parent: Failed to create pool");

    let tensor = pool
        .alloc(&[4], TensorDtype::U32)
        .expect("Parent: Failed to allocate tensor");

    // Write magic values to GPU
    let values: [u32; 4] = [0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x87654321];
    let gpu_ptr = pool.device_ptr(&tensor);

    memcpy(
        gpu_ptr,
        values.as_ptr() as *const std::ffi::c_void,
        16,
        CudaMemcpyKind::HostToDevice,
    )
    .expect("Parent: memcpy failed");
    device_synchronize().expect("Parent: sync failed");

    // Get IPC handle as hex string
    let handle_hex: String = tensor
        .ipc_handle_bytes()
        .iter()
        .map(|b| format!("{:02x}", b))
        .collect();

    println!("Parent: Spawning child process...");

    // Spawn child process
    let exe = std::env::current_exe().unwrap();
    let mut child = Command::new(exe)
        .arg("--test")
        .arg("test_cuda_ipc_cross_process")
        .arg("--ignored")
        .env("HORUS_IPC_TEST_CHILD", "1")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to spawn child");

    // Send IPC handle to child
    child
        .stdin
        .as_mut()
        .unwrap()
        .write_all(format!("{}\n", handle_hex).as_bytes())
        .expect("Failed to write to child stdin");

    // Wait for child
    let output = child.wait_with_output().expect("Failed to wait for child");

    if !output.status.success() {
        eprintln!("Child stderr: {}", String::from_utf8_lossy(&output.stderr));
        panic!("Child process failed");
    }

    println!("Parent: Child verified GPU memory successfully!");
    println!("Cross-process CUDA IPC test passed!");

    pool.release(&tensor).unwrap();
}

#[cfg(test)]
mod cuda_ffi_tests {
    #[test]
    fn test_device_count() {
        use horus_core::memory::cuda_ffi::get_device_count;

        match get_device_count() {
            Ok(count) => {
                println!("Found {} CUDA device(s)", count);
                assert!(count >= 0);
            }
            Err(e) => {
                println!("CUDA not available: {}", e);
            }
        }
    }

    #[test]
    fn test_set_get_device() {
        use horus_core::memory::cuda_available;
        use horus_core::memory::cuda_ffi::{get_device, set_device};

        if !cuda_available() {
            return;
        }

        set_device(0).expect("Failed to set device 0");
        let device = get_device().expect("Failed to get device");
        assert_eq!(device, 0);
    }
}
