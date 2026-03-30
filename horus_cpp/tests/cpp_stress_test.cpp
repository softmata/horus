// C++ Stress Test — rapid scheduler create/destroy + tick cycles
//
// Compile normally: g++ -std=c++17 -o cpp_stress_test tests/cpp_stress_test.cpp -L target/debug -lhorus_cpp -lpthread -ldl -lm
// Compile with ASAN: g++ -std=c++17 -fsanitize=address -o cpp_stress_asan tests/cpp_stress_test.cpp -L target/debug -lhorus_cpp -lpthread -ldl -lm
// Run: LD_LIBRARY_PATH=target/debug ./cpp_stress_test

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

int main() {
    std::printf("=== HORUS C++ Stress Test ===\n\n");
    auto start = std::chrono::steady_clock::now();

    // Test 1: Rapid create/destroy (leak detection)
    {
        const int ITERATIONS = 1000;
        std::printf("Test 1: %d scheduler create/destroy cycles... ", ITERATIONS);
        for (int i = 0; i < ITERATIONS; i++) {
            void* sched = horus_scheduler_new();
            horus_scheduler_tick_rate(sched, 100.0);
            horus_scheduler_destroy(sched);
        }
        std::printf("PASS\n");
    }

    // Test 2: Sustained ticking
    {
        const int TICKS = 5000;
        std::printf("Test 2: %d sustained ticks with callback node... ", TICKS);

        void* sched = horus_scheduler_new();
        void* builder = horus_node_builder_new("stress_node");
        horus_node_builder_set_tick(builder, stress_tick);
        horus_node_builder_build(builder, sched);

        total_ticks = 0;
        for (int i = 0; i < TICKS; i++) {
            horus_scheduler_tick_once(sched);
        }

        std::printf("PASS (ticked %d times)\n", total_ticks);
        horus_scheduler_destroy(sched);
    }

    // Test 3: Multiple nodes
    {
        const int NODE_COUNT = 50;
        std::printf("Test 3: %d nodes in one scheduler... ", NODE_COUNT);

        void* sched = horus_scheduler_new();
        for (int i = 0; i < NODE_COUNT; i++) {
            char name[32];
            std::snprintf(name, sizeof(name), "node_%d", i);
            void* builder = horus_node_builder_new(name);
            horus_node_builder_set_tick(builder, stress_tick);
            horus_node_builder_build(builder, sched);
        }

        total_ticks = 0;
        for (int i = 0; i < 100; i++) {
            horus_scheduler_tick_once(sched);
        }

        std::printf("PASS (%d total ticks across %d nodes)\n", total_ticks, NODE_COUNT);
        horus_scheduler_destroy(sched);
    }

    // Test 4: Null robustness under stress
    {
        const int ITERATIONS = 10000;
        std::printf("Test 4: %d null-pointer calls... ", ITERATIONS);
        for (int i = 0; i < ITERATIONS; i++) {
            horus_scheduler_is_running(nullptr);
            horus_scheduler_tick_once(nullptr);
            horus_scheduler_stop(nullptr);
            horus_scheduler_tick_rate(nullptr, 100.0);
        }
        std::printf("PASS\n");
    }

    auto elapsed = std::chrono::steady_clock::now() - start;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    std::printf("\n=== All stress tests passed in %ldms ===\n", ms);
    return 0;
}
