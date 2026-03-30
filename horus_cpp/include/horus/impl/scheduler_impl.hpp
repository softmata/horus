// HORUS Scheduler Implementation — inline, header-only.
// Wraps the C API (horus_c.h) in idiomatic C++ classes.
#pragma once

#include "../horus_c.h"
#include "../scheduler.hpp"
#include "../node.hpp"
#include "../duration.hpp"
#include "../error.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

namespace horus {

// ─── Scheduler ───────────────────────────────────────────────────────────────

inline Scheduler::Scheduler() : inner_(horus_scheduler_new()) {
    if (!inner_) throw Error("Failed to create scheduler");
}

inline Scheduler::~Scheduler() {
    if (inner_) horus_scheduler_destroy(inner_);
}

inline Scheduler::Scheduler(Scheduler&& other) noexcept : inner_(other.inner_) {
    other.inner_ = nullptr;
}

inline Scheduler& Scheduler::operator=(Scheduler&& other) noexcept {
    if (this != &other) {
        if (inner_) horus_scheduler_destroy(inner_);
        inner_ = other.inner_;
        other.inner_ = nullptr;
    }
    return *this;
}

inline Scheduler& Scheduler::tick_rate(Frequency freq) {
    horus_scheduler_tick_rate(inner_, freq.value());
    return *this;
}

inline Scheduler& Scheduler::name(std::string_view n) {
    std::string s(n);
    horus_scheduler_name(inner_, s.c_str());
    return *this;
}

inline Scheduler& Scheduler::prefer_rt() {
    horus_scheduler_prefer_rt(inner_);
    return *this;
}

inline Scheduler& Scheduler::require_rt() {
    horus_scheduler_require_rt(inner_);
    return *this;
}

inline Scheduler& Scheduler::deterministic(bool enabled) {
    horus_scheduler_deterministic(inner_, enabled);
    return *this;
}

inline Scheduler& Scheduler::verbose(bool enabled) {
    horus_scheduler_verbose(inner_, enabled);
    return *this;
}

inline Scheduler& Scheduler::watchdog(Duration timeout) {
    horus_scheduler_watchdog(inner_, static_cast<uint64_t>(timeout.count()));
    return *this;
}

inline Scheduler& Scheduler::blackbox(size_t size_mb) {
    horus_scheduler_blackbox(inner_, size_mb);
    return *this;
}

inline Scheduler& Scheduler::enable_network() {
    horus_scheduler_enable_network(inner_);
    return *this;
}

inline NodeBuilder Scheduler::add(std::string_view n) {
    return NodeBuilder(this, n);
}

inline NodeBuilder Scheduler::add(Node& node) {
    auto builder = NodeBuilder(this, node.name());
    builder.tick([&node] { node.tick(); });
    builder.init([&node] { node.init(); });
    builder.safe_state([&node] { node.enter_safe_state(); });
    return builder;
}

inline NodeBuilder Scheduler::add(LambdaNode& node) {
    node.call_init();
    auto builder = NodeBuilder(this, node.name());
    builder.tick([&node] {
        node.call_tick();
    });
    return builder;
}

inline void Scheduler::spin() {
    if (horus_scheduler_tick_once(inner_) != 0) {
        // For now, spin = repeated tick_once. Real spin() would call scheduler_run().
    }
    while (is_running()) {
        horus_scheduler_tick_once(inner_);
    }
}

inline void Scheduler::tick_once() {
    if (horus_scheduler_tick_once(inner_) != 0) {
        throw Error("tick_once failed");
    }
}

inline void Scheduler::stop() {
    horus_scheduler_stop(inner_);
}

inline bool Scheduler::is_running() const {
    return horus_scheduler_is_running(inner_);
}

inline std::string Scheduler::get_name() const {
    char buf[256];
    int len = horus_scheduler_get_name(inner_, buf, sizeof(buf));
    return len > 0 ? std::string(buf, static_cast<size_t>(len)) : std::string();
}

inline std::string Scheduler::status() const {
    return is_running() ? "running" : "stopped";
}

inline bool Scheduler::has_full_rt() const {
    return horus_scheduler_has_full_rt(inner_);
}

inline std::vector<std::string> Scheduler::node_list() const {
    uint32_t count = horus_scheduler_node_count(inner_);
    std::vector<std::string> result;
    result.reserve(count);
    char buf[256];
    for (uint32_t i = 0; i < count; i++) {
        int len = horus_scheduler_node_name_at(inner_, i, buf, sizeof(buf));
        if (len > 0) result.emplace_back(buf, static_cast<size_t>(len));
    }
    return result;
}

// ─── NodeBuilder ─────────────────────────────────────────────────────────────

inline NodeBuilder::NodeBuilder(Scheduler* sched, std::string_view n)
    : sched_(sched), tick_fn_(nullptr)
{
    std::string s(n);
    inner_ = horus_node_builder_new(s.c_str());
}

inline NodeBuilder& NodeBuilder::rate(Frequency freq) {
    horus_node_builder_rate(
        inner_, freq.value());
    return *this;
}

inline NodeBuilder& NodeBuilder::budget(Duration dur) {
    horus_node_builder_budget(
        inner_,
        static_cast<uint64_t>(dur.count()));
    return *this;
}

inline NodeBuilder& NodeBuilder::deadline(Duration dur) {
    horus_node_builder_deadline(
        inner_,
        static_cast<uint64_t>(dur.count()));
    return *this;
}

inline NodeBuilder& NodeBuilder::on_miss(Miss policy) {
    horus_node_builder_on_miss(
        inner_,
        static_cast<uint8_t>(policy));
    return *this;
}

inline NodeBuilder& NodeBuilder::compute() {
    horus_node_builder_compute(inner_);
    return *this;
}

inline NodeBuilder& NodeBuilder::async_io() {
    horus_node_builder_async_io(inner_);
    return *this;
}

inline NodeBuilder& NodeBuilder::on(std::string_view topic) {
    std::string s(topic);
    horus_node_builder_on_topic(inner_, s.c_str());
    return *this;
}

inline NodeBuilder& NodeBuilder::order(uint32_t ord) {
    horus_node_builder_order(inner_, ord);
    return *this;
}

inline NodeBuilder& NodeBuilder::pin_core(size_t cpu_id) {
    horus_node_builder_pin_core(inner_, cpu_id);
    return *this;
}

inline NodeBuilder& NodeBuilder::priority(int32_t prio) {
    horus_node_builder_priority(inner_, prio);
    return *this;
}

inline NodeBuilder& NodeBuilder::watchdog(Duration timeout) {
    horus_node_builder_watchdog(inner_, static_cast<uint64_t>(timeout.count()));
    return *this;
}

inline NodeBuilder& NodeBuilder::tick(std::function<void()> callback) {
    tick_fn_ = std::move(callback);
    return *this;
}

inline NodeBuilder& NodeBuilder::init(std::function<void()> callback) {
    init_fn_ = std::move(callback);
    return *this;
}

inline NodeBuilder& NodeBuilder::safe_state(std::function<void()> callback) {
    safe_state_fn_ = std::move(callback);
    return *this;
}

// Per-node callback registry. Each node gets a unique slot.
// The extern "C" trampoline indexes into this array.
// Per-node callback registry with trampolines for tick, init, and safe_state.
namespace detail {
    static constexpr size_t MAX_NODES = 256;
    static std::function<void()> g_tick_callbacks[MAX_NODES];
    static std::function<void()> g_init_callbacks[MAX_NODES];
    static std::function<void()> g_safe_callbacks[MAX_NODES];
    static size_t g_next_slot = 0;

    // Generate trampolines for tick, init, and safe_state
    #define HORUS_TRAMPOLINE_TICK(N) inline void tick_trampoline_##N() { if (g_tick_callbacks[N]) g_tick_callbacks[N](); }
    #define HORUS_TRAMPOLINE_INIT(N) inline void init_trampoline_##N() { if (g_init_callbacks[N]) g_init_callbacks[N](); }
    #define HORUS_TRAMPOLINE_SAFE(N) inline void safe_trampoline_##N() { if (g_safe_callbacks[N]) g_safe_callbacks[N](); }

    #define HORUS_ALL_TRAMPOLINES(N) HORUS_TRAMPOLINE_TICK(N) HORUS_TRAMPOLINE_INIT(N) HORUS_TRAMPOLINE_SAFE(N)

    HORUS_ALL_TRAMPOLINES(0)  HORUS_ALL_TRAMPOLINES(1)  HORUS_ALL_TRAMPOLINES(2)  HORUS_ALL_TRAMPOLINES(3)
    HORUS_ALL_TRAMPOLINES(4)  HORUS_ALL_TRAMPOLINES(5)  HORUS_ALL_TRAMPOLINES(6)  HORUS_ALL_TRAMPOLINES(7)
    HORUS_ALL_TRAMPOLINES(8)  HORUS_ALL_TRAMPOLINES(9)  HORUS_ALL_TRAMPOLINES(10) HORUS_ALL_TRAMPOLINES(11)
    HORUS_ALL_TRAMPOLINES(12) HORUS_ALL_TRAMPOLINES(13) HORUS_ALL_TRAMPOLINES(14) HORUS_ALL_TRAMPOLINES(15)
    HORUS_ALL_TRAMPOLINES(16) HORUS_ALL_TRAMPOLINES(17) HORUS_ALL_TRAMPOLINES(18) HORUS_ALL_TRAMPOLINES(19)
    HORUS_ALL_TRAMPOLINES(20) HORUS_ALL_TRAMPOLINES(21) HORUS_ALL_TRAMPOLINES(22) HORUS_ALL_TRAMPOLINES(23)
    HORUS_ALL_TRAMPOLINES(24) HORUS_ALL_TRAMPOLINES(25) HORUS_ALL_TRAMPOLINES(26) HORUS_ALL_TRAMPOLINES(27)
    HORUS_ALL_TRAMPOLINES(28) HORUS_ALL_TRAMPOLINES(29) HORUS_ALL_TRAMPOLINES(30) HORUS_ALL_TRAMPOLINES(31)

    using TrampolineFn = void(*)();

    #define HORUS_TABLE_ENTRY(N) tick_trampoline_##N
    inline TrampolineFn get_tick_trampoline(size_t slot) {
        static const TrampolineFn table[] = {
            HORUS_TABLE_ENTRY(0),  HORUS_TABLE_ENTRY(1),  HORUS_TABLE_ENTRY(2),  HORUS_TABLE_ENTRY(3),
            HORUS_TABLE_ENTRY(4),  HORUS_TABLE_ENTRY(5),  HORUS_TABLE_ENTRY(6),  HORUS_TABLE_ENTRY(7),
            HORUS_TABLE_ENTRY(8),  HORUS_TABLE_ENTRY(9),  HORUS_TABLE_ENTRY(10), HORUS_TABLE_ENTRY(11),
            HORUS_TABLE_ENTRY(12), HORUS_TABLE_ENTRY(13), HORUS_TABLE_ENTRY(14), HORUS_TABLE_ENTRY(15),
            HORUS_TABLE_ENTRY(16), HORUS_TABLE_ENTRY(17), HORUS_TABLE_ENTRY(18), HORUS_TABLE_ENTRY(19),
            HORUS_TABLE_ENTRY(20), HORUS_TABLE_ENTRY(21), HORUS_TABLE_ENTRY(22), HORUS_TABLE_ENTRY(23),
            HORUS_TABLE_ENTRY(24), HORUS_TABLE_ENTRY(25), HORUS_TABLE_ENTRY(26), HORUS_TABLE_ENTRY(27),
            HORUS_TABLE_ENTRY(28), HORUS_TABLE_ENTRY(29), HORUS_TABLE_ENTRY(30), HORUS_TABLE_ENTRY(31),
        };
        return slot < 32 ? table[slot] : nullptr;
    }
    #undef HORUS_TABLE_ENTRY

    #define HORUS_TABLE_ENTRY(N) init_trampoline_##N
    inline TrampolineFn get_init_trampoline(size_t slot) {
        static const TrampolineFn table[] = {
            HORUS_TABLE_ENTRY(0),  HORUS_TABLE_ENTRY(1),  HORUS_TABLE_ENTRY(2),  HORUS_TABLE_ENTRY(3),
            HORUS_TABLE_ENTRY(4),  HORUS_TABLE_ENTRY(5),  HORUS_TABLE_ENTRY(6),  HORUS_TABLE_ENTRY(7),
            HORUS_TABLE_ENTRY(8),  HORUS_TABLE_ENTRY(9),  HORUS_TABLE_ENTRY(10), HORUS_TABLE_ENTRY(11),
            HORUS_TABLE_ENTRY(12), HORUS_TABLE_ENTRY(13), HORUS_TABLE_ENTRY(14), HORUS_TABLE_ENTRY(15),
            HORUS_TABLE_ENTRY(16), HORUS_TABLE_ENTRY(17), HORUS_TABLE_ENTRY(18), HORUS_TABLE_ENTRY(19),
            HORUS_TABLE_ENTRY(20), HORUS_TABLE_ENTRY(21), HORUS_TABLE_ENTRY(22), HORUS_TABLE_ENTRY(23),
            HORUS_TABLE_ENTRY(24), HORUS_TABLE_ENTRY(25), HORUS_TABLE_ENTRY(26), HORUS_TABLE_ENTRY(27),
            HORUS_TABLE_ENTRY(28), HORUS_TABLE_ENTRY(29), HORUS_TABLE_ENTRY(30), HORUS_TABLE_ENTRY(31),
        };
        return slot < 32 ? table[slot] : nullptr;
    }
    #undef HORUS_TABLE_ENTRY

    #define HORUS_TABLE_ENTRY(N) safe_trampoline_##N
    inline TrampolineFn get_safe_trampoline(size_t slot) {
        static const TrampolineFn table[] = {
            HORUS_TABLE_ENTRY(0),  HORUS_TABLE_ENTRY(1),  HORUS_TABLE_ENTRY(2),  HORUS_TABLE_ENTRY(3),
            HORUS_TABLE_ENTRY(4),  HORUS_TABLE_ENTRY(5),  HORUS_TABLE_ENTRY(6),  HORUS_TABLE_ENTRY(7),
            HORUS_TABLE_ENTRY(8),  HORUS_TABLE_ENTRY(9),  HORUS_TABLE_ENTRY(10), HORUS_TABLE_ENTRY(11),
            HORUS_TABLE_ENTRY(12), HORUS_TABLE_ENTRY(13), HORUS_TABLE_ENTRY(14), HORUS_TABLE_ENTRY(15),
            HORUS_TABLE_ENTRY(16), HORUS_TABLE_ENTRY(17), HORUS_TABLE_ENTRY(18), HORUS_TABLE_ENTRY(19),
            HORUS_TABLE_ENTRY(20), HORUS_TABLE_ENTRY(21), HORUS_TABLE_ENTRY(22), HORUS_TABLE_ENTRY(23),
            HORUS_TABLE_ENTRY(24), HORUS_TABLE_ENTRY(25), HORUS_TABLE_ENTRY(26), HORUS_TABLE_ENTRY(27),
            HORUS_TABLE_ENTRY(28), HORUS_TABLE_ENTRY(29), HORUS_TABLE_ENTRY(30), HORUS_TABLE_ENTRY(31),
        };
        return slot < 32 ? table[slot] : nullptr;
    }
    #undef HORUS_TABLE_ENTRY

    #undef HORUS_TRAMPOLINE_TICK
    #undef HORUS_TRAMPOLINE_INIT
    #undef HORUS_TRAMPOLINE_SAFE
    #undef HORUS_ALL_TRAMPOLINES
}

inline void NodeBuilder::build() {
    if (tick_fn_ || init_fn_ || safe_state_fn_) {
        size_t slot = detail::g_next_slot++;
        if (slot >= 32) {
            throw Error("Too many nodes (max 32 with callbacks)");
        }

        if (tick_fn_) {
            detail::g_tick_callbacks[slot] = std::move(tick_fn_);
            horus_node_builder_set_tick(inner_, detail::get_tick_trampoline(slot));
        }
        if (init_fn_) {
            detail::g_init_callbacks[slot] = std::move(init_fn_);
            horus_node_builder_set_init(inner_, detail::get_init_trampoline(slot));
        }
        if (safe_state_fn_) {
            detail::g_safe_callbacks[slot] = std::move(safe_state_fn_);
            horus_node_builder_set_safe_state(inner_, detail::get_safe_trampoline(slot));
        }
    }
    int result = horus_node_builder_build(inner_, sched_->inner_);
    inner_ = nullptr;
    if (result != 0) {
        throw Error("Failed to build node");
    }
}

} // namespace horus
