// Hand-written C++ mirror of the Rust diagnostics message types.
//
// Source of truth: horus_types::* — /home/neos/softmata/horus/horus_types/src/diagnostics.rs
// (NOT horus-robotics; the previous banner here claimed this file was generated
//  from horus-robotics/messages/diagnostics.rs. Both claims were false, and that
//  is precisely how the layouts below were allowed to drift out of sync.)
//
// The C ABI (`horus_publisher_*_send` / `horus_subscriber_*_recv`) performs a raw
// `std::ptr::read` / `std::ptr::write` of `size_of::<RustType>()` bytes through the
// pointer C++ hands it. These structs MUST therefore be byte-for-byte identical to
// their Rust counterparts — same field order, same widths, same padding.
// `horus/layout_contract.hpp` pins that with static_asserts; include it to be checked.
//
// Every struct here is a plain aggregate (standard-layout POD): no constructors,
// no methods, no inheritance. Rust `u8` fields that are logically booleans are
// mirrored as `uint8_t`, not `bool`, because the Rust type is literally `u8`.
#pragma once

#include <cstddef>
#include <cstdint>

namespace horus { namespace msg {

/// System heartbeat sent over Topic IPC.
/// Mirrors Rust `horus_types::Heartbeat` (72 bytes).
///
/// Not to be confused with `NodeHeartbeat`, which is a filesystem-based
/// monitoring record with epoch-second timestamps.
struct Heartbeat {
    uint8_t  node_name[32];   // offset 0   — node name (null-terminated)
    uint32_t node_id;         // offset 32  — numeric node ID
    uint64_t sequence;        // offset 40  — increments each heartbeat
    uint8_t  alive;           // offset 48  — Rust u8, 0/1
    double   uptime;          // offset 56  — seconds since startup
    uint64_t timestamp_ns;    // offset 64  — nanoseconds since epoch
};

/// General-purpose component status report.
/// Mirrors Rust `horus_types::DiagnosticStatus` (176 bytes).
struct DiagnosticStatus {
    uint8_t  level;           // offset 0   — StatusLevel: 0=Ok, 1=Warn, 2=Error, 3=Fatal
    uint32_t code;            // offset 4   — component-specific status/error code
    uint8_t  message[128];    // offset 8   — human-readable (null-terminated)
    uint8_t  component[32];   // offset 136 — reporting component (null-terminated)
    uint64_t timestamp_ns;    // offset 168 — nanoseconds since epoch
};

/// Critical safety message: stop all robot motion immediately.
/// Mirrors Rust `horus_types::EmergencyStop` (112 bytes).
struct EmergencyStop {
    uint8_t  engaged;         // offset 0   — Rust u8, 0/1
    uint8_t  reason[64];      // offset 1   — null-terminated
    uint8_t  source[32];      // offset 65  — null-terminated
    uint8_t  auto_reset;      // offset 97  — Rust u8, 0/1
    uint64_t timestamp_ns;    // offset 104 — nanoseconds since epoch
};

/// System resource utilization snapshot.
/// Mirrors Rust `horus_types::ResourceUsage` (72 bytes).
struct ResourceUsage {
    float    cpu_percent;        // offset 0  — 0-100
    uint64_t memory_bytes;       // offset 8
    float    memory_percent;     // offset 16 — 0-100
    uint64_t disk_bytes;         // offset 24
    float    disk_percent;       // offset 32 — 0-100
    uint64_t network_tx_bytes;   // offset 40
    uint64_t network_rx_bytes;   // offset 48
    float    temperature;        // offset 56 — celsius
    uint32_t thread_count;       // offset 60
    uint64_t timestamp_ns;       // offset 64 — nanoseconds since epoch
};

/// Node status heartbeat written to the filesystem heartbeats directory
/// for external monitoring tools. Epoch-SECOND timestamps (not nanoseconds).
/// Mirrors Rust `horus_types::NodeHeartbeat` (48 bytes).
struct NodeHeartbeat {
    uint8_t  state;                 // offset 0  — NodeStateMsg discriminant
    uint8_t  health;                // offset 1  — horus_core::core::HealthStatus discriminant
    uint64_t tick_count;            // offset 8
    uint32_t target_rate_hz;        // offset 16
    uint32_t actual_rate_hz;        // offset 20 — measured
    uint32_t error_count;           // offset 24
    uint64_t last_tick_timestamp;   // offset 32 — unix epoch SECONDS
    uint64_t heartbeat_timestamp;   // offset 40 — unix epoch SECONDS
};

/// Safety system status flags.
/// Mirrors Rust `horus_types::SafetyStatus` (24 bytes).
struct SafetyStatus {
    uint8_t  enabled;         // offset 0 — safety system active (Rust u8)
    uint8_t  estop_engaged;   // offset 1 — Rust u8
    uint8_t  watchdog_ok;     // offset 2 — Rust u8
    uint8_t  limits_ok;       // offset 3 — Rust u8
    uint8_t  comms_ok;        // offset 4 — Rust u8
    uint8_t  mode;            // offset 5 — 0=normal, 1=reduced, 2=safe_stop
    uint32_t fault_code;      // offset 8
    uint64_t timestamp_ns;    // offset 16 — nanoseconds since epoch
};

/// Single diagnostic key-value pair.
/// Mirrors Rust `horus_types::DiagnosticValue` (97 bytes, alignment 1).
///
/// NOTE: absent from the generated `layout_contract.hpp`, so no static_assert
/// pins it, and it has no `impl_pod_topic_c_api!` entry point in c_api.rs — so
/// there is no live FFI read/write surface today. The layout below was verified
/// empirically against the Rust struct (97 bytes; key@0, value@32, value_type@96)
/// and will not be caught by the build if either side drifts.
struct DiagnosticValue {
    uint8_t key[32];          // offset 0  — null-terminated
    uint8_t value[64];        // offset 32 — null-terminated
    uint8_t value_type;       // offset 96 — 0=string, 1=int, 2=float, 3=bool
};

/// Diagnostic report carrying up to 16 key-value pairs.
/// Mirrors Rust `horus_types::DiagnosticReport` (1600 bytes).
///
/// NOTE: as with `DiagnosticValue` — absent from `layout_contract.hpp` and from
/// c_api.rs, so unpinned but with no live FFI surface. Layout verified
/// empirically against the Rust struct (1600 bytes; component@0, values@32,
/// value_count@1584, level@1585, timestamp_ns@1592).
struct DiagnosticReport {
    uint8_t         component[32];  // offset 0    — null-terminated
    DiagnosticValue values[16];     // offset 32   — 16 * 97 bytes
    uint8_t         value_count;    // offset 1584 — number of valid entries
    uint8_t         level;          // offset 1585 — StatusLevel discriminant
    uint64_t        timestamp_ns;   // offset 1592 — nanoseconds since epoch
};

}} // namespace horus::msg
