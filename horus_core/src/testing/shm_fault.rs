//! SHM fault injection utilities for testing error recovery paths.
//!
//! Provides controlled corruption of shared memory regions to test that
//! HORUS safely handles corrupted headers, truncated data, stale epochs,
//! and invalid slot states without panicking or causing undefined behavior.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::testing::shm_fault::{ShmFaultInjector, ShmFault};
//!
//! let injector = ShmFaultInjector::new("test_topic", 4096).unwrap();
//! injector.inject(ShmFault::CorruptHeader);
//! // Now attempt operations on the corrupted SHM — they should return
//! // errors or None, never panic.
//! ```

use crate::error::{HorusError, HorusResult};
use crate::memory::platform::shm_topics_dir;
use crate::memory::shm_region::ShmRegion;
use std::path::PathBuf;

/// Types of faults that can be injected into shared memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShmFault {
    /// Overwrite the header region with garbage bytes (0xDE pattern).
    CorruptHeader,
    /// Set the epoch counter to u64::MAX (stale/invalid epoch).
    StaleEpoch,
    /// Truncate data by zeroing everything past the header region.
    TruncatedData,
    /// Set all slot ready flags to invalid values.
    InvalidSlotFlags,
    /// Fill the entire SHM region with zeros (simulates uninitialized memory).
    ZeroFill,
    /// Fill with a repeating byte pattern (simulates memory corruption).
    PatternFill(u8),
    /// Corrupt a specific byte range within the SHM region.
    CorruptRange {
        offset: usize,
        len: usize,
        pattern: u8,
    },
}

/// Utility for injecting faults into HORUS shared memory regions.
///
/// Creates a real SHM region (backed by `/dev/shm` on Linux) and provides
/// methods to corrupt it in controlled ways for testing error recovery.
pub struct ShmFaultInjector {
    region: ShmRegion,
    size: usize,
    #[cfg(target_os = "linux")]
    shm_path: PathBuf,
}

impl ShmFaultInjector {
    /// Create a new SHM region for fault injection testing.
    ///
    /// The region is created with the given name and size. It starts
    /// zeroed (clean state).
    pub fn new(name: &str, size: usize) -> HorusResult<Self> {
        #[cfg(target_os = "linux")]
        let shm_path = shm_topics_dir().join(format!("horus_{}", name));
        let region = ShmRegion::new(name, size)?;
        Ok(Self {
            region,
            size,
            #[cfg(target_os = "linux")]
            shm_path,
        })
    }

    /// Inject a specific fault into the SHM region.
    pub fn inject(&self, fault: ShmFault) {
        let ptr = self.region.as_ptr() as *mut u8;

        // SAFETY: We own this SHM region and it's sized to `self.size`.
        // Writes are within bounds. This is explicitly for testing.
        unsafe {
            match fault {
                ShmFault::CorruptHeader => {
                    // Corrupt the first 128 bytes (typical header size)
                    let header_size = 128.min(self.size);
                    for i in 0..header_size {
                        ptr.add(i).write_volatile(0xDE);
                    }
                }
                ShmFault::StaleEpoch => {
                    // Epoch is typically stored in the header at a known offset.
                    // Write u64::MAX to the first 8 bytes (common epoch location).
                    if self.size >= 8 {
                        let epoch_ptr = ptr as *mut u64;
                        epoch_ptr.write_volatile(u64::MAX);
                    }
                }
                ShmFault::TruncatedData => {
                    // Keep header (first 128 bytes) but zero everything else
                    let data_start = 128.min(self.size);
                    for i in data_start..self.size {
                        ptr.add(i).write_volatile(0);
                    }
                }
                ShmFault::InvalidSlotFlags => {
                    // Slot ready flags are typically 8-byte values at slot boundaries.
                    // Set them all to 0xFF (invalid state).
                    let slot_size = 64; // typical minimum slot size
                    let mut offset = 128; // skip header
                    while offset + 8 <= self.size {
                        let flag_ptr = ptr.add(offset) as *mut u64;
                        flag_ptr.write_volatile(0xFFFF_FFFF_FFFF_FFFF);
                        offset += slot_size;
                    }
                }
                ShmFault::ZeroFill => {
                    for i in 0..self.size {
                        ptr.add(i).write_volatile(0);
                    }
                }
                ShmFault::PatternFill(pattern) => {
                    for i in 0..self.size {
                        ptr.add(i).write_volatile(pattern);
                    }
                }
                ShmFault::CorruptRange {
                    offset,
                    len,
                    pattern,
                } => {
                    let end = (offset + len).min(self.size);
                    let start = offset.min(self.size);
                    for i in start..end {
                        ptr.add(i).write_volatile(pattern);
                    }
                }
            }
        }
    }

    /// Read raw bytes from the SHM region (for verification in tests).
    pub fn read_bytes(&self, offset: usize, len: usize) -> Option<Vec<u8>> {
        if offset + len > self.size {
            return None;
        }
        let ptr = self.region.as_ptr();
        let mut result = vec![0u8; len];
        // SAFETY: Bounds checked above. Read-only access to owned region.
        unsafe {
            for (i, byte) in result.iter_mut().enumerate() {
                *byte = ptr.add(offset + i).read_volatile();
            }
        }
        Some(result)
    }

    /// Write raw bytes to the SHM region.
    pub fn write_bytes(&self, offset: usize, data: &[u8]) -> HorusResult<()> {
        if offset + data.len() > self.size {
            return Err(HorusError::InvalidInput(format!(
                "write at offset {} + len {} exceeds region size {}",
                offset,
                data.len(),
                self.size
            )));
        }
        let ptr = self.region.as_ptr() as *mut u8;
        // SAFETY: Bounds checked above. Write access to owned region.
        unsafe {
            for (i, &byte) in data.iter().enumerate() {
                ptr.add(offset + i).write_volatile(byte);
            }
        }
        Ok(())
    }

    /// Get the raw pointer to the SHM region (for advanced test scenarios).
    pub fn as_ptr(&self) -> *const u8 {
        self.region.as_ptr()
    }

    /// Get the size of the SHM region.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get the path to the backing SHM file (Linux only).
    #[cfg(target_os = "linux")]
    pub fn path(&self) -> &PathBuf {
        &self.shm_path
    }
}

/// Simulated network fault modes for testing mDNS and peer discovery.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetworkFault {
    /// Simulate a timeout (no response received within deadline).
    Timeout,
    /// Simulate an unreachable peer (connection refused).
    Unreachable,
    /// Simulate a DNS resolution failure.
    DnsFailure(String),
    /// Simulate a corrupted response (invalid data received).
    CorruptResponse,
}

/// Mock network endpoint for testing discovery and connection error paths.
///
/// Simulates network failures without requiring real network access.
/// Use this to test that HORUS handles mDNS timeouts, unreachable peers,
/// and DNS failures gracefully.
pub struct MockNetworkEndpoint {
    hostname: String,
    fault: Option<NetworkFault>,
    resolve_count: std::sync::atomic::AtomicU64,
    connect_count: std::sync::atomic::AtomicU64,
}

impl MockNetworkEndpoint {
    /// Create an endpoint that always succeeds.
    pub fn healthy(hostname: impl Into<String>) -> Self {
        Self {
            hostname: hostname.into(),
            fault: None,
            resolve_count: std::sync::atomic::AtomicU64::new(0),
            connect_count: std::sync::atomic::AtomicU64::new(0),
        }
    }

    /// Create an endpoint with a specific fault mode.
    pub fn with_fault(hostname: impl Into<String>, fault: NetworkFault) -> Self {
        Self {
            hostname: hostname.into(),
            fault: Some(fault),
            resolve_count: std::sync::atomic::AtomicU64::new(0),
            connect_count: std::sync::atomic::AtomicU64::new(0),
        }
    }

    /// Simulate a hostname resolution attempt.
    ///
    /// Returns `Ok(ip)` for healthy endpoints, `Err` for faulted ones.
    pub fn resolve(&self) -> HorusResult<std::net::IpAddr> {
        self.resolve_count
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

        match &self.fault {
            None => Ok(std::net::IpAddr::V4(std::net::Ipv4Addr::LOCALHOST)),
            Some(NetworkFault::Timeout) => Err(HorusError::Timeout(format!(
                "mDNS resolution timeout for {}",
                self.hostname
            ))),
            Some(NetworkFault::Unreachable) => Err(HorusError::communication(format!(
                "peer {} unreachable",
                self.hostname
            ))),
            Some(NetworkFault::DnsFailure(reason)) => Err(HorusError::communication(format!(
                "DNS resolution failed for {}: {}",
                self.hostname, reason
            ))),
            Some(NetworkFault::CorruptResponse) => Err(HorusError::communication(format!(
                "corrupt mDNS response from {}",
                self.hostname
            ))),
        }
    }

    /// Simulate a connection attempt to this endpoint.
    ///
    /// Returns `Ok(())` for healthy endpoints, `Err` for faulted ones.
    pub fn connect(&self) -> HorusResult<()> {
        self.connect_count
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);

        match &self.fault {
            None => Ok(()),
            Some(NetworkFault::Timeout) => Err(HorusError::Timeout(format!(
                "connection timeout to {}",
                self.hostname
            ))),
            Some(NetworkFault::Unreachable) => Err(HorusError::communication(format!(
                "connection refused by {}",
                self.hostname
            ))),
            Some(NetworkFault::DnsFailure(reason)) => Err(HorusError::communication(format!(
                "cannot connect to {}: DNS failure ({})",
                self.hostname, reason
            ))),
            Some(NetworkFault::CorruptResponse) => Err(HorusError::communication(format!(
                "received corrupt handshake from {}",
                self.hostname
            ))),
        }
    }

    /// Get the hostname.
    pub fn hostname(&self) -> &str {
        &self.hostname
    }

    /// Get total resolution attempts.
    pub fn resolve_attempts(&self) -> u64 {
        self.resolve_count
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Get total connection attempts.
    pub fn connect_attempts(&self) -> u64 {
        self.connect_count
            .load(std::sync::atomic::Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── SHM Fault Injection Tests ──

    #[test]
    fn shm_fault_corrupt_header_overwrites_first_128_bytes() {
        let injector = ShmFaultInjector::new("test_fault_header", 4096).unwrap();
        injector.inject(ShmFault::CorruptHeader);

        let header = injector.read_bytes(0, 128).unwrap();
        assert!(
            header.iter().all(|&b| b == 0xDE),
            "Header bytes must be overwritten with 0xDE"
        );

        // Data region should be untouched (still zeros from init)
        let data = injector.read_bytes(128, 64).unwrap();
        assert!(
            data.iter().all(|&b| b == 0),
            "Data after header should be untouched"
        );
    }

    #[test]
    fn shm_fault_stale_epoch_writes_max_u64() {
        let injector = ShmFaultInjector::new("test_fault_epoch", 4096).unwrap();
        injector.inject(ShmFault::StaleEpoch);

        let epoch_bytes = injector.read_bytes(0, 8).unwrap();
        let epoch = u64::from_ne_bytes(epoch_bytes.try_into().unwrap());
        assert_eq!(epoch, u64::MAX, "Epoch must be set to u64::MAX");
    }

    #[test]
    fn shm_fault_truncated_data_zeros_after_header() {
        let injector = ShmFaultInjector::new("test_fault_trunc", 4096).unwrap();

        // First, write some data everywhere
        injector.inject(ShmFault::PatternFill(0xAB));
        // Then truncate
        injector.inject(ShmFault::TruncatedData);

        // Header should still have 0xAB
        let header = injector.read_bytes(0, 128).unwrap();
        assert!(
            header.iter().all(|&b| b == 0xAB),
            "Header should be preserved"
        );

        // Data after header should be zeroed
        let data = injector.read_bytes(128, 256).unwrap();
        assert!(
            data.iter().all(|&b| b == 0),
            "Data after header should be zeroed"
        );
    }

    #[test]
    fn shm_fault_zero_fill_clears_entire_region() {
        let injector = ShmFaultInjector::new("test_fault_zero", 4096).unwrap();
        injector.inject(ShmFault::PatternFill(0xFF));
        injector.inject(ShmFault::ZeroFill);

        let all = injector.read_bytes(0, 4096).unwrap();
        assert!(all.iter().all(|&b| b == 0), "Entire region must be zeroed");
    }

    #[test]
    fn shm_fault_pattern_fill_writes_everywhere() {
        let injector = ShmFaultInjector::new("test_fault_pattern", 4096).unwrap();
        injector.inject(ShmFault::PatternFill(0xCA));

        let sample = injector.read_bytes(0, 4096).unwrap();
        assert!(
            sample.iter().all(|&b| b == 0xCA),
            "Entire region must be filled with 0xCA"
        );
    }

    #[test]
    fn shm_fault_corrupt_range_targeted() {
        let injector = ShmFaultInjector::new("test_fault_range", 4096).unwrap();
        injector.inject(ShmFault::CorruptRange {
            offset: 100,
            len: 50,
            pattern: 0xBB,
        });

        // Before range should be clean
        let before = injector.read_bytes(90, 10).unwrap();
        assert!(
            before.iter().all(|&b| b == 0),
            "Bytes before corrupt range should be clean"
        );

        // Corrupt range
        let corrupt = injector.read_bytes(100, 50).unwrap();
        assert!(
            corrupt.iter().all(|&b| b == 0xBB),
            "Corrupt range must contain 0xBB"
        );

        // After range should be clean
        let after = injector.read_bytes(150, 10).unwrap();
        assert!(
            after.iter().all(|&b| b == 0),
            "Bytes after corrupt range should be clean"
        );
    }

    #[test]
    fn shm_fault_corrupt_range_clamped_to_size() {
        let injector = ShmFaultInjector::new("test_fault_clamp", 256).unwrap();
        // Requesting beyond region size should be clamped, not panic
        injector.inject(ShmFault::CorruptRange {
            offset: 200,
            len: 200, // would exceed 256 bytes
            pattern: 0xCC,
        });

        let data = injector.read_bytes(200, 56).unwrap();
        assert!(
            data.iter().all(|&b| b == 0xCC),
            "Clamped range should be corrupted"
        );
    }

    #[test]
    fn shm_fault_write_and_read_bytes() {
        let injector = ShmFaultInjector::new("test_fault_rw", 4096).unwrap();
        let data = vec![1, 2, 3, 4, 5];
        injector.write_bytes(100, &data).unwrap();

        let read_back = injector.read_bytes(100, 5).unwrap();
        assert_eq!(read_back, data);
    }

    #[test]
    fn shm_fault_write_out_of_bounds_returns_error() {
        let injector = ShmFaultInjector::new("test_fault_oob", 256).unwrap();
        let result = injector.write_bytes(250, &[0; 10]);
        assert!(result.is_err(), "Writing past region size should fail");
    }

    #[test]
    fn shm_fault_read_out_of_bounds_returns_none() {
        let injector = ShmFaultInjector::new("test_fault_read_oob", 256).unwrap();
        let result = injector.read_bytes(250, 10);
        assert!(
            result.is_none(),
            "Reading past region size should return None"
        );
    }

    // ── Network Fault Tests ──

    #[test]
    fn network_healthy_endpoint_resolves() {
        let endpoint = MockNetworkEndpoint::healthy("robot.local");
        let result = endpoint.resolve();
        assert!(result.is_ok());
        assert_eq!(endpoint.resolve_attempts(), 1);
    }

    #[test]
    fn network_healthy_endpoint_connects() {
        let endpoint = MockNetworkEndpoint::healthy("robot.local");
        let result = endpoint.connect();
        assert!(result.is_ok());
        assert_eq!(endpoint.connect_attempts(), 1);
    }

    #[test]
    fn network_timeout_fault_on_resolve() {
        let endpoint = MockNetworkEndpoint::with_fault("slow.local", NetworkFault::Timeout);
        let result = endpoint.resolve();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            format!("{}", err).contains("timeout"),
            "Error should mention timeout: {}",
            err
        );
    }

    #[test]
    fn network_timeout_fault_on_connect() {
        let endpoint = MockNetworkEndpoint::with_fault("slow.local", NetworkFault::Timeout);
        let result = endpoint.connect();
        assert!(result.is_err());
    }

    #[test]
    fn network_unreachable_fault() {
        let endpoint = MockNetworkEndpoint::with_fault("offline.local", NetworkFault::Unreachable);
        let result = endpoint.resolve();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            format!("{}", err).contains("unreachable"),
            "Error should mention unreachable: {}",
            err
        );
    }

    #[test]
    fn network_dns_failure_with_reason() {
        let endpoint = MockNetworkEndpoint::with_fault(
            "bad.local",
            NetworkFault::DnsFailure("NXDOMAIN".to_string()),
        );
        let result = endpoint.resolve();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            format!("{}", err).contains("NXDOMAIN"),
            "Error should include DNS reason: {}",
            err
        );
    }

    #[test]
    fn network_corrupt_response_fault() {
        let endpoint =
            MockNetworkEndpoint::with_fault("corrupt.local", NetworkFault::CorruptResponse);
        let result = endpoint.connect();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            format!("{}", err).contains("corrupt"),
            "Error should mention corruption: {}",
            err
        );
    }

    #[test]
    fn network_endpoint_tracks_attempts() {
        let endpoint = MockNetworkEndpoint::with_fault("fail.local", NetworkFault::Timeout);
        for _ in 0..5 {
            let _ = endpoint.resolve();
        }
        for _ in 0..3 {
            let _ = endpoint.connect();
        }
        assert_eq!(endpoint.resolve_attempts(), 5);
        assert_eq!(endpoint.connect_attempts(), 3);
    }

    #[test]
    fn network_hostname_preserved() {
        let endpoint = MockNetworkEndpoint::healthy("my-robot.local");
        assert_eq!(endpoint.hostname(), "my-robot.local");
    }
}
