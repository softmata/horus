// HORUS C++ Scheduler — builder pattern for real-time scheduling.
//
// Usage:
//   using namespace horus::literals;
//   auto sched = horus::Scheduler()
//       .tick_rate(100_hz)
//       .prefer_rt()
//       .monitoring(true);
//
//   sched.add("controller")
//       .rate(50_hz)
//       .budget(5_ms)
//       .on_miss(horus::Miss::Skip)
//       .tick([&] { /* ... */ })
//       .build();
//
//   sched.spin();
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "duration.hpp"
#include "error.hpp"

// Forward declarations — opaque types from horus_c.h
struct HorusScheduler;
struct HorusNodeBuilder;

namespace horus {

class NodeBuilder;

/// Real-time scheduler — creates, configures, and runs nodes.
///
/// Builder methods return `*this` for chaining. Configuration is
/// deferred until `spin()` / `tick_once()`.
class Scheduler {
public:
    Scheduler();
    ~Scheduler();

    // Move-only (owns Rust Box<FfiScheduler>)
    Scheduler(Scheduler&& other) noexcept;
    Scheduler& operator=(Scheduler&& other) noexcept;
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;

    /// Set the scheduler tick rate.
    Scheduler& tick_rate(Frequency freq);

    /// Set the scheduler name.
    Scheduler& name(std::string_view name);

    /// Prefer real-time scheduling (graceful degradation if unavailable).
    Scheduler& prefer_rt();

    /// Require real-time scheduling (fail if unavailable).
    Scheduler& require_rt();

    /// Enable or disable deterministic mode (SimClock + seeded RNG).
    Scheduler& deterministic(bool enabled);

    /// Enable or disable verbose logging.
    Scheduler& verbose(bool enabled);

    /// Set the global watchdog timeout.
    Scheduler& watchdog(Duration timeout);

    /// Set BlackBox flight recorder size in MB.
    Scheduler& blackbox(size_t size_mb);

    /// Enable LAN network replication.
    Scheduler& enable_network();

    /// Add a node — returns a NodeBuilder for configuration.
    NodeBuilder add(std::string_view name);

    /// Run the scheduler (blocks until stopped or error).
    void spin();

    /// Execute a single tick of all nodes.
    void tick_once();

    /// Stop the scheduler (can be called from any thread).
    void stop();

    /// Check if the scheduler is still running.
    bool is_running() const;

    /// Get the scheduler name.
    std::string get_name() const;

    /// Get a human-readable status string.
    std::string status() const;

    /// Check if full RT capabilities are available.
    bool has_full_rt() const;

    /// Get list of registered node names.
    std::vector<std::string> node_list() const;

private:
    friend class NodeBuilder;
    HorusScheduler* inner_;  // Owned Rust Box (released in destructor)
};

/// Node builder — configures a node before adding to the scheduler.
///
/// Usage:
///   sched.add("ctrl")
///       .rate(100_hz)
///       .budget(1_ms)
///       .on_miss(Miss::Skip)
///       .tick([&] { /* ... */ })
///       .build();
class NodeBuilder {
public:
    NodeBuilder& rate(Frequency freq);
    NodeBuilder& budget(Duration dur);
    NodeBuilder& deadline(Duration dur);
    NodeBuilder& on_miss(Miss policy);
    NodeBuilder& compute();
    NodeBuilder& async_io();
    NodeBuilder& on(std::string_view topic);
    NodeBuilder& order(uint32_t ord);
    NodeBuilder& pin_core(size_t cpu_id);
    NodeBuilder& priority(int32_t prio);
    NodeBuilder& watchdog(Duration timeout);

    /// Set the tick callback — called each scheduler tick.
    NodeBuilder& tick(std::function<void()> callback);

    /// Finalize and register the node with the scheduler.
    void build();

private:
    friend class Scheduler;
    NodeBuilder(Scheduler* sched, std::string_view name);

    Scheduler* sched_;
    HorusNodeBuilder* inner_;  // Owned Rust Box (consumed by build())
    std::function<void()> tick_fn_;
};

} // namespace horus
