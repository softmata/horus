// C++ End-to-End Test — calls HORUS through the C API.
// Uses raw extern "C" declarations (no ergonomic header) to verify the FFI
// surface works standalone. Converted to GoogleTest.

#include <gtest/gtest.h>
#include <chrono>
#include <cstdint>
#include <thread>

// Forward-declare the C API (matches c_api.rs #[no_mangle] extern "C" functions)
extern "C" {
    // Scheduler
    void* horus_scheduler_new();
    void  horus_scheduler_destroy(void* sched);
    void  horus_scheduler_tick_rate(void* sched, double hz);
    void  horus_scheduler_prefer_rt(void* sched);
    void  horus_scheduler_stop(const void* sched);
    bool  horus_scheduler_is_running(const void* sched);
    int   horus_scheduler_tick_once(void* sched);

    // Node builder
    void* horus_node_builder_new(const char* name);
    void  horus_node_builder_rate(void* builder, double hz);
    void  horus_node_builder_set_tick(void* builder, void (*callback)());
    int   horus_node_builder_build(void* builder, void* sched);

    // Version
    uint32_t horus_get_abi_version();
}

// Test tick callback (C-linkage for pointer-to-function FFI)
static int tick_count = 0;
extern "C" void test_tick() { tick_count++; }

TEST(Cabi, AbiVersionAtLeast1) {
    uint32_t ver = horus_get_abi_version();
    EXPECT_GE(ver, 1u) << "ABI version must be >= 1";
}

TEST(Cabi, SchedulerLifecycle) {
    void* sched = horus_scheduler_new();
    ASSERT_NE(sched, nullptr);

    EXPECT_TRUE(horus_scheduler_is_running(sched)) << "Scheduler running after creation";

    horus_scheduler_tick_rate(sched, 100.0);
    horus_scheduler_prefer_rt(sched);

    int result = horus_scheduler_tick_once(sched);
    EXPECT_EQ(result, 0) << "tick_once on empty scheduler";

    horus_scheduler_stop(sched);
    EXPECT_FALSE(horus_scheduler_is_running(sched)) << "Scheduler stopped";

    horus_scheduler_destroy(sched);
    // If we got here, destroy did not crash
    SUCCEED();
}

TEST(Cabi, NodeTickCallbackInvoked) {
    void* sched = horus_scheduler_new();
    ASSERT_NE(sched, nullptr);
    void* builder = horus_node_builder_new("cpp_test_node");
    ASSERT_NE(builder, nullptr);

    // Don't set rate — keeps as BestEffort (runs on main thread, synchronous)
    horus_node_builder_set_tick(builder, test_tick);

    int result = horus_node_builder_build(builder, sched);
    EXPECT_EQ(result, 0) << "Node built successfully";

    tick_count = 0;
    horus_scheduler_tick_once(sched);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_GE(tick_count, 1) << "C++ tick callback invoked";

    horus_scheduler_destroy(sched);
}

TEST(Cabi, NullSafety) {
    // None of these must crash
    horus_scheduler_destroy(nullptr);
    horus_scheduler_tick_rate(nullptr, 100.0);
    horus_scheduler_stop(nullptr);
    EXPECT_FALSE(horus_scheduler_is_running(nullptr)) << "null is_running returns false";
}
