// C++ End-to-End Test — calls HORUS through the C API
//
// Compile: g++ -std=c++17 -o cpp_e2e_test tests/cpp_e2e_test.cpp -L target/debug -lhorus_cpp -lpthread -ldl -lm
// Run:     LD_LIBRARY_PATH=target/debug ./cpp_e2e_test

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
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

// Test tick callback
static int tick_count = 0;
extern "C" void test_tick() {
    tick_count++;
}

int main() {
    std::printf("=== HORUS C++ E2E Test ===\n\n");
    int passed = 0;
    int failed = 0;

    // Test 1: ABI version
    {
        uint32_t ver = horus_get_abi_version();
        if (ver >= 1) {
            std::printf("[PASS] ABI version: %u\n", ver);
            passed++;
        } else {
            std::printf("[FAIL] ABI version: %u (expected >= 1)\n", ver);
            failed++;
        }
    }

    // Test 2: Scheduler lifecycle
    {
        void* sched = horus_scheduler_new();
        assert(sched != nullptr);

        bool running = horus_scheduler_is_running(sched);
        if (running) {
            std::printf("[PASS] Scheduler is running after creation\n");
            passed++;
        } else {
            std::printf("[FAIL] Scheduler not running\n");
            failed++;
        }

        horus_scheduler_tick_rate(sched, 100.0);
        horus_scheduler_prefer_rt(sched);

        int result = horus_scheduler_tick_once(sched);
        if (result == 0) {
            std::printf("[PASS] tick_once succeeded (empty scheduler)\n");
            passed++;
        } else {
            std::printf("[FAIL] tick_once returned %d\n", result);
            failed++;
        }

        horus_scheduler_stop(sched);
        if (!horus_scheduler_is_running(sched)) {
            std::printf("[PASS] Scheduler stopped\n");
            passed++;
        } else {
            std::printf("[FAIL] Scheduler still running after stop\n");
            failed++;
        }

        horus_scheduler_destroy(sched);
        std::printf("[PASS] Scheduler destroyed without crash\n");
        passed++;
    }

    // Test 3: Node with tick callback
    {
        void* sched = horus_scheduler_new();
        void* builder = horus_node_builder_new("cpp_test_node");
        assert(builder != nullptr);

        // Don't set rate — keeps as BestEffort (runs on main thread, synchronous)
        horus_node_builder_set_tick(builder, test_tick);

        int result = horus_node_builder_build(builder, sched);
        if (result == 0) {
            std::printf("[PASS] Node built successfully\n");
            passed++;
        } else {
            std::printf("[FAIL] Node build returned %d\n", result);
            failed++;
        }

        // Tick and verify callback was invoked
        // RT nodes run on separate threads — give them a moment
        tick_count = 0;
        horus_scheduler_tick_once(sched);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (tick_count >= 1) {
            std::printf("[PASS] C++ tick callback invoked (%d times)\n", tick_count);
            passed++;
        } else {
            std::printf("[FAIL] tick_count = %d (expected >= 1)\n", tick_count);
            failed++;
        }

        horus_scheduler_destroy(sched);
    }

    // Test 4: Null safety
    {
        horus_scheduler_destroy(nullptr);  // should not crash
        horus_scheduler_tick_rate(nullptr, 100.0);  // should not crash
        horus_scheduler_stop(nullptr);  // should not crash
        bool running = horus_scheduler_is_running(nullptr);
        if (!running) {
            std::printf("[PASS] Null safety — all null calls handled\n");
            passed++;
        } else {
            std::printf("[FAIL] Null safety issue\n");
            failed++;
        }
    }

    // Summary
    std::printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
