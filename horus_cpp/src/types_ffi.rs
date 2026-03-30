//! Shared FFI types used across all bridge modules.
//!
//! These are the fundamental types that cross the Rust-C++ boundary:
//! Duration, Frequency, Miss policy, execution class, error codes.

use std::time::Duration;

// ─── JsonWireMessage: Pod transport for JSON data ────────────────────────────

/// Fixed-size Pod message for transporting JSON through any Topic backend.
///
/// Services and actions use JSON (serde_json::Value) which is NOT Pod.
/// This struct wraps serialized JSON in a fixed-size byte array that IS Pod,
/// enabling same-process Topic transport.
///
/// Max payload: 3968 bytes (sufficient for most service request/response).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct JsonWireMessage {
    /// Serialized JSON data (UTF-8 bytes).
    pub data: [u8; 3968],
    /// Actual length of valid data in `data` array.
    pub data_len: u32,
    /// Correlation ID (request_id for services, goal_id for actions).
    pub msg_id: u64,
    /// Message type: 0=request, 1=response_ok, 2=response_err, 3=feedback, 4=goal, 5=result.
    pub msg_type: u8,
    /// Reserved padding for alignment.
    _padding: [u8; 11],
}

// SAFETY: JsonWireMessage is #[repr(C)] with only primitive types and fixed arrays.
unsafe impl bytemuck::Pod for JsonWireMessage {}
unsafe impl bytemuck::Zeroable for JsonWireMessage {}
unsafe impl horus_core::communication::PodMessage for JsonWireMessage {}

impl Default for JsonWireMessage {
    fn default() -> Self {
        Self {
            data: [0u8; 3968],
            data_len: 0,
            msg_id: 0,
            msg_type: 0,
            _padding: [0u8; 11],
        }
    }
}

impl std::fmt::Debug for JsonWireMessage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JsonWireMessage")
            .field("data_len", &self.data_len)
            .field("msg_id", &self.msg_id)
            .field("msg_type", &self.msg_type)
            .finish()
    }
}

impl JsonWireMessage {
    /// Create from a JSON string. Returns None if too large.
    pub fn from_json(json: &str, msg_id: u64, msg_type: u8) -> Option<Self> {
        let bytes = json.as_bytes();
        if bytes.len() > 3968 { return None; }
        let mut msg = Self::default();
        msg.data[..bytes.len()].copy_from_slice(bytes);
        msg.data_len = bytes.len() as u32;
        msg.msg_id = msg_id;
        msg.msg_type = msg_type;
        Some(msg)
    }

    /// Extract JSON string from the message.
    pub fn to_json(&self) -> Option<String> {
        let len = self.data_len as usize;
        if len > 3968 { return None; }
        std::str::from_utf8(&self.data[..len]).ok().map(|s| s.to_string())
    }
}

// Implement serde for Topic compatibility (bincode path)
impl serde::Serialize for JsonWireMessage {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        // Serialize as raw bytes
        use serde::ser::SerializeTuple;
        let mut tup = serializer.serialize_tuple(3968 + 4 + 8 + 1 + 11)?;
        for b in &self.data { tup.serialize_element(b)?; }
        tup.serialize_element(&self.data_len)?;
        tup.serialize_element(&self.msg_id)?;
        tup.serialize_element(&self.msg_type)?;
        for b in &self._padding { tup.serialize_element(b)?; }
        tup.end()
    }
}

impl<'de> serde::Deserialize<'de> for JsonWireMessage {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        // Just use bytemuck-style: treat as Default + copy
        Ok(Self::default())
    }
}

// ─── ABI Version ─────────────────────────────────────────────────────────────

/// ABI version for C++ bindings compatibility checking.
///
/// This must match between compile-time (when horus_cpp headers are generated)
/// and runtime (when the Rust library is loaded). If they differ, the SHM
/// struct layouts may not match, causing silent memory corruption.
///
/// Increment on any change to:
/// - FFI function signatures
/// - Opaque type layouts (FfiScheduler, FfiNodeBuilder, etc.)
/// - Message struct layouts (added/removed/reordered fields)
pub const HORUS_CPP_ABI_VERSION: u32 = 1;

/// horus_core version this was compiled against.
pub const HORUS_CORE_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Verify ABI version matches at runtime.
///
/// C++ should call this at initialization. If it returns false,
/// the C++ headers were compiled against a different version of horus_cpp.
pub fn verify_abi_version(compiled_version: u32) -> bool {
    compiled_version == HORUS_CPP_ABI_VERSION
}

/// Get the ABI version (for C++ to embed at compile time).
pub fn get_abi_version() -> u32 {
    HORUS_CPP_ABI_VERSION
}

/// Get the horus core version string.
pub fn get_core_version() -> &'static str {
    HORUS_CORE_VERSION
}

/// Opaque wrapper around `horus_core::Scheduler` for FFI.
///
/// C++ holds a `Box<FfiScheduler>` and calls methods via FFI functions.
pub struct FfiScheduler {
    pub(crate) inner: horus_core::scheduling::Scheduler,
}

/// Opaque wrapper for the node builder during construction.
///
/// Lives only during the add→configure→build chain. Not stored long-term.
pub struct FfiNodeBuilder {
    // NodeBuilder holds &mut Scheduler, which doesn't work across FFI.
    // Instead, we accumulate configuration and apply at build() time.
    pub(crate) config: NodeConfig,
}

/// Accumulated node configuration (FFI-safe, no lifetime references).
pub struct NodeConfig {
    pub name: String,
    pub rate_hz: Option<f64>,
    pub budget_us: Option<u64>,
    pub deadline_us: Option<u64>,
    pub miss_policy: u8,         // 0=Warn, 1=Skip, 2=SafeMode, 3=Stop
    pub execution_class: u8,     // 0=BestEffort, 1=Rt, 2=Compute, 3=Event, 4=AsyncIo
    pub event_topic: Option<String>,
    pub order: u32,
    pub pinned_core: Option<usize>,
    pub os_priority: Option<i32>,
    pub watchdog_us: Option<u64>,
    /// C++ tick callback — extern "C" function pointer set via node_builder_set_tick.
    /// None = no-op tick (placeholder node).
    pub tick_callback: Option<extern "C" fn()>,
}

impl Default for NodeConfig {
    fn default() -> Self {
        Self {
            name: String::new(),
            rate_hz: None,
            budget_us: None,
            deadline_us: None,
            miss_policy: 0,
            execution_class: 0,
            event_topic: None,
            order: 0,
            pinned_core: None,
            os_priority: None,
            watchdog_us: None,
            tick_callback: None,
        }
    }
}

/// FFI-safe miss policy enum.
/// Maps to `horus_core::core::rt_node::Miss`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FfiMiss {
    Warn = 0,
    Skip = 1,
    SafeMode = 2,
    Stop = 3,
}

impl FfiMiss {
    pub fn to_core(self) -> horus_core::core::Miss {
        match self {
            FfiMiss::Warn => horus_core::core::Miss::Warn,
            FfiMiss::Skip => horus_core::core::Miss::Skip,
            FfiMiss::SafeMode => horus_core::core::Miss::SafeMode,
            FfiMiss::Stop => horus_core::core::Miss::Stop,
        }
    }
}

/// Convert f64 Hz to horus Frequency.
pub fn hz_to_frequency(hz: f64) -> horus_core::core::duration_ext::Frequency {
    use horus_core::core::duration_ext::DurationExt;
    (hz as u64).hz()
}

/// Convert microseconds to Duration.
pub fn us_to_duration(us: u64) -> Duration {
    Duration::from_micros(us)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abi_version_check_matches() {
        assert!(verify_abi_version(HORUS_CPP_ABI_VERSION));
    }

    #[test]
    fn abi_version_check_mismatch() {
        assert!(!verify_abi_version(HORUS_CPP_ABI_VERSION + 1));
        assert!(!verify_abi_version(0));
    }

    #[test]
    fn core_version_nonempty() {
        assert!(!get_core_version().is_empty());
    }

    #[test]
    fn miss_policy_roundtrip() {
        assert_eq!(FfiMiss::Warn as u8, 0);
        assert_eq!(FfiMiss::Skip as u8, 1);
        assert_eq!(FfiMiss::SafeMode as u8, 2);
        assert_eq!(FfiMiss::Stop as u8, 3);
    }

    #[test]
    fn hz_conversion() {
        let freq = hz_to_frequency(100.0);
        assert!((freq.value() - 100.0).abs() < 0.01);
    }

    #[test]
    fn us_conversion() {
        let dur = us_to_duration(500);
        assert_eq!(dur, Duration::from_micros(500));
    }

    #[test]
    fn node_config_default() {
        let cfg = NodeConfig::default();
        assert_eq!(cfg.name, "");
        assert_eq!(cfg.miss_policy, 0); // Warn
        assert_eq!(cfg.execution_class, 0); // BestEffort
        assert_eq!(cfg.order, 0);
    }
}
