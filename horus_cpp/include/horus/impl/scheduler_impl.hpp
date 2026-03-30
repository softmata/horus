// HORUS Scheduler Implementation — inline, header-only.
// Wraps the C API (horus_c.h) in idiomatic C++ classes.
#pragma once

#include "../horus_c.h"
#include "../scheduler.hpp"
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

// Per-node callback registry. Each node gets a unique slot.
// The extern "C" trampoline indexes into this array.
namespace detail {
    static constexpr size_t MAX_NODES = 256;
    static std::function<void()> g_node_callbacks[MAX_NODES];
    static size_t g_next_slot = 0;

    // Each trampoline must be a plain C function (extern "C" compatible).
    // We generate them via a macro since extern "C" can't be templated.
    #define HORUS_TRAMPOLINE(N) \
        inline void node_trampoline_##N() { \
            if (g_node_callbacks[N]) g_node_callbacks[N](); \
        }

    HORUS_TRAMPOLINE(0)  HORUS_TRAMPOLINE(1)  HORUS_TRAMPOLINE(2)  HORUS_TRAMPOLINE(3)
    HORUS_TRAMPOLINE(4)  HORUS_TRAMPOLINE(5)  HORUS_TRAMPOLINE(6)  HORUS_TRAMPOLINE(7)
    HORUS_TRAMPOLINE(8)  HORUS_TRAMPOLINE(9)  HORUS_TRAMPOLINE(10) HORUS_TRAMPOLINE(11)
    HORUS_TRAMPOLINE(12) HORUS_TRAMPOLINE(13) HORUS_TRAMPOLINE(14) HORUS_TRAMPOLINE(15)
    HORUS_TRAMPOLINE(16) HORUS_TRAMPOLINE(17) HORUS_TRAMPOLINE(18) HORUS_TRAMPOLINE(19)
    HORUS_TRAMPOLINE(20) HORUS_TRAMPOLINE(21) HORUS_TRAMPOLINE(22) HORUS_TRAMPOLINE(23)
    HORUS_TRAMPOLINE(24) HORUS_TRAMPOLINE(25) HORUS_TRAMPOLINE(26) HORUS_TRAMPOLINE(27)
    HORUS_TRAMPOLINE(28) HORUS_TRAMPOLINE(29) HORUS_TRAMPOLINE(30) HORUS_TRAMPOLINE(31)

    using TrampolineFn = void(*)();
    inline TrampolineFn get_trampoline(size_t slot) {
        static const TrampolineFn table[] = {
            node_trampoline_0,  node_trampoline_1,  node_trampoline_2,  node_trampoline_3,
            node_trampoline_4,  node_trampoline_5,  node_trampoline_6,  node_trampoline_7,
            node_trampoline_8,  node_trampoline_9,  node_trampoline_10, node_trampoline_11,
            node_trampoline_12, node_trampoline_13, node_trampoline_14, node_trampoline_15,
            node_trampoline_16, node_trampoline_17, node_trampoline_18, node_trampoline_19,
            node_trampoline_20, node_trampoline_21, node_trampoline_22, node_trampoline_23,
            node_trampoline_24, node_trampoline_25, node_trampoline_26, node_trampoline_27,
            node_trampoline_28, node_trampoline_29, node_trampoline_30, node_trampoline_31,
        };
        if (slot < 32) return table[slot];
        return nullptr;
    }
    #undef HORUS_TRAMPOLINE
}

inline void NodeBuilder::build() {
    if (tick_fn_) {
        size_t slot = detail::g_next_slot++;
        if (slot >= 32) {
            throw Error("Too many nodes (max 32 with tick callbacks)");
        }
        detail::g_node_callbacks[slot] = std::move(tick_fn_);
        auto trampoline = detail::get_trampoline(slot);
        if (!trampoline) {
            throw Error("Failed to allocate trampoline slot");
        }
        horus_node_builder_set_tick(inner_, trampoline);
    }
    int result = horus_node_builder_build(inner_, sched_->inner_);
    inner_ = nullptr;
    if (result != 0) {
        throw Error("Failed to build node");
    }
}

} // namespace horus
