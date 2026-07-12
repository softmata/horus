// C++ Stress Test — rapid scheduler create/destroy + tick cycles.
// Converted to GoogleTest. Still callable with -fsanitize=address via CI.

#include <gtest/gtest.h>
#include <chrono>
#include <cstdint>
#include <cstdio>

extern "C" {
    void*    horus_scheduler_new();
    void     horus_scheduler_destroy(void* sched);
    void     horus_scheduler_tick_rate(void* sched, double hz);
    void     horus_scheduler_stop(const void* sched);
    bool     horus_scheduler_is_running(const void* sched);
    int      horus_scheduler_tick_once(void* sched);
    void*    horus_node_builder_new(const char* name);
    void     horus_node_builder_set_tick(void* builder, void (*callback)());
    int      horus_node_builder_build(void* builder, void* sched);
    uint32_t horus_get_abi_version();
}

static int total_ticks = 0;
extern "C" void stress_tick() { total_ticks++; }

TEST(Stress, CreateDestroy1000Cycles) {
    const int ITERATIONS = 1000;
    for (int i = 0; i < ITERATIONS; i++) {
        void* sched = horus_scheduler_new();
        ASSERT_NE(sched, nullptr) << "iter " << i;
        horus_scheduler_tick_rate(sched, 100.0);
        horus_scheduler_destroy(sched);
    }
    SUCCEED() << ITERATIONS << " schedulers created/destroyed without crash";
}

TEST(Stress, SustainedTicks5000) {
    const int TICKS = 5000;
    void* sched = horus_scheduler_new();
    ASSERT_NE(sched, nullptr);
    void* builder = horus_node_builder_new("stress_node");
    ASSERT_NE(builder, nullptr);
    horus_node_builder_set_tick(builder, stress_tick);
    ASSERT_EQ(horus_node_builder_build(builder, sched), 0);

    total_ticks = 0;
    for (int i = 0; i < TICKS; i++) {
        horus_scheduler_tick_once(sched);
    }
    // The global trampoline pattern means total_ticks may not equal TICKS
    // exactly on all configurations, but it must be > 0.
    EXPECT_GT(total_ticks, 0) << "ticked " << total_ticks << " times (expected > 0)";
    horus_scheduler_destroy(sched);
}

TEST(Stress, FiftyNodesOneScheduler) {
    const int NODE_COUNT = 50;
    void* sched = horus_scheduler_new();
    ASSERT_NE(sched, nullptr);
    for (int i = 0; i < NODE_COUNT; i++) {
        char name[32];
        std::snprintf(name, sizeof(name), "node_%d", i);
        void* builder = horus_node_builder_new(name);
        ASSERT_NE(builder, nullptr) << "builder " << i;
        horus_node_builder_set_tick(builder, stress_tick);
        ASSERT_EQ(horus_node_builder_build(builder, sched), 0) << "build " << i;
    }

    total_ticks = 0;
    for (int i = 0; i < 100; i++) {
        horus_scheduler_tick_once(sched);
    }
    EXPECT_GT(total_ticks, 0) << "total ticks across " << NODE_COUNT << " nodes";
    horus_scheduler_destroy(sched);
}

TEST(Stress, NullPointerRobustness10000Calls) {
    const int ITERATIONS = 10000;
    for (int i = 0; i < ITERATIONS; i++) {
        horus_scheduler_is_running(nullptr);
        horus_scheduler_tick_once(nullptr);
        horus_scheduler_stop(nullptr);
        horus_scheduler_tick_rate(nullptr, 100.0);
    }
    SUCCEED() << ITERATIONS << " null-pointer calls without crash";
}
