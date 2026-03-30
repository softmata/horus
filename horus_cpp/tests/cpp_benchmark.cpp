// HORUS C++ FFI Benchmark Suite
//
// Measures real-world performance of C++ calling Rust through the FFI boundary.
// All measurements are from the C++ side — this is what users actually experience.
//
// Compile: g++ -std=c++17 -O2 -o cpp_benchmark tests/cpp_benchmark.cpp -L target/release -lhorus_cpp -lpthread -ldl -lm
// Run:     LD_LIBRARY_PATH=target/release ./cpp_benchmark

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <vector>

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

using Clock = std::chrono::high_resolution_clock;
using ns = std::chrono::nanoseconds;

// ─── Statistics ──────────────────────────────────────────────────────────────

struct Stats {
    double min_ns;
    double max_ns;
    double mean_ns;
    double median_ns;
    double p99_ns;
    double p999_ns;
    double stddev_ns;
    size_t count;
};

Stats compute_stats(std::vector<int64_t>& samples) {
    std::sort(samples.begin(), samples.end());
    size_t n = samples.size();

    double sum = 0;
    for (auto s : samples) sum += s;
    double mean = sum / n;

    double var = 0;
    for (auto s : samples) var += (s - mean) * (s - mean);
    var /= n;

    return Stats {
        .min_ns    = static_cast<double>(samples[0]),
        .max_ns    = static_cast<double>(samples[n - 1]),
        .mean_ns   = mean,
        .median_ns = static_cast<double>(samples[n / 2]),
        .p99_ns    = static_cast<double>(samples[(size_t)(n * 0.99)]),
        .p999_ns   = static_cast<double>(samples[(size_t)(n * 0.999)]),
        .stddev_ns = std::sqrt(var),
        .count     = n,
    };
}

void print_stats(const char* name, const Stats& s) {
    std::printf("  %-30s  min=%6.0f  mean=%7.1f  median=%6.0f  p99=%7.0f  p999=%8.0f  max=%8.0f  stddev=%6.1f  (n=%zu)\n",
        name, s.min_ns, s.mean_ns, s.median_ns, s.p99_ns, s.p999_ns, s.max_ns, s.stddev_ns, s.count);
}

// ─── Benchmarks ──────────────────────────────────────────────────────────────

void bench_scheduler_create_destroy(int iterations) {
    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 100; i++) {
        void* s = horus_scheduler_new();
        horus_scheduler_destroy(s);
    }

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        void* s = horus_scheduler_new();
        horus_scheduler_destroy(s);
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
    }

    auto stats = compute_stats(samples);
    print_stats("scheduler create+destroy", stats);
}

void bench_tick_once_empty(int iterations) {
    void* sched = horus_scheduler_new();
    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 100; i++) horus_scheduler_tick_once(sched);

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        horus_scheduler_tick_once(sched);
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
    }

    horus_scheduler_destroy(sched);
    auto stats = compute_stats(samples);
    print_stats("tick_once (empty scheduler)", stats);
}

static volatile int sink = 0;
extern "C" void bench_tick_cb() { sink++; }

void bench_tick_once_1_node(int iterations) {
    void* sched = horus_scheduler_new();
    void* b = horus_node_builder_new("bench_node");
    horus_node_builder_set_tick(b, bench_tick_cb);
    horus_node_builder_build(b, sched);

    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 100; i++) horus_scheduler_tick_once(sched);

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        horus_scheduler_tick_once(sched);
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
    }

    horus_scheduler_destroy(sched);
    auto stats = compute_stats(samples);
    print_stats("tick_once (1 node + callback)", stats);
}

void bench_tick_once_10_nodes(int iterations) {
    void* sched = horus_scheduler_new();
    for (int i = 0; i < 10; i++) {
        char name[32];
        std::snprintf(name, sizeof(name), "node_%d", i);
        void* b = horus_node_builder_new(name);
        horus_node_builder_set_tick(b, bench_tick_cb);
        horus_node_builder_build(b, sched);
    }

    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 100; i++) horus_scheduler_tick_once(sched);

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        horus_scheduler_tick_once(sched);
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
    }

    horus_scheduler_destroy(sched);
    auto stats = compute_stats(samples);
    print_stats("tick_once (10 nodes)", stats);
}

void bench_tick_once_50_nodes(int iterations) {
    void* sched = horus_scheduler_new();
    for (int i = 0; i < 50; i++) {
        char name[32];
        std::snprintf(name, sizeof(name), "node_%d", i);
        void* b = horus_node_builder_new(name);
        horus_node_builder_set_tick(b, bench_tick_cb);
        horus_node_builder_build(b, sched);
    }

    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 100; i++) horus_scheduler_tick_once(sched);

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        horus_scheduler_tick_once(sched);
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
    }

    horus_scheduler_destroy(sched);
    auto stats = compute_stats(samples);
    print_stats("tick_once (50 nodes)", stats);
}

void bench_ffi_call_overhead(int iterations) {
    // Measure the bare cost of crossing the FFI boundary
    // horus_get_abi_version() is the simplest possible FFI call (no args, returns u32)
    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 1000; i++) horus_get_abi_version();

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        volatile uint32_t v = horus_get_abi_version();
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
        (void)v;
    }

    auto stats = compute_stats(samples);
    print_stats("FFI call overhead (abi_version)", stats);
}

void bench_is_running(int iterations) {
    void* sched = horus_scheduler_new();
    std::vector<int64_t> samples(iterations);

    // Warmup
    for (int i = 0; i < 1000; i++) horus_scheduler_is_running(sched);

    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        volatile bool r = horus_scheduler_is_running(sched);
        auto t1 = Clock::now();
        samples[i] = std::chrono::duration_cast<ns>(t1 - t0).count();
        (void)r;
    }

    horus_scheduler_destroy(sched);
    auto stats = compute_stats(samples);
    print_stats("is_running (atomic read)", stats);
}

void bench_throughput() {
    // Measure maximum ticks/second with 1 node
    void* sched = horus_scheduler_new();
    void* b = horus_node_builder_new("throughput_node");
    horus_node_builder_set_tick(b, bench_tick_cb);
    horus_node_builder_build(b, sched);

    // Warmup
    for (int i = 0; i < 1000; i++) horus_scheduler_tick_once(sched);

    sink = 0;
    auto t0 = Clock::now();
    const int TICKS = 100000;
    for (int i = 0; i < TICKS; i++) {
        horus_scheduler_tick_once(sched);
    }
    auto t1 = Clock::now();
    double elapsed_s = std::chrono::duration_cast<ns>(t1 - t0).count() / 1e9;
    double ticks_per_sec = TICKS / elapsed_s;

    horus_scheduler_destroy(sched);
    std::printf("  %-30s  %.0f ticks/sec  (%.1f us/tick, %d ticks in %.3fs)\n",
        "throughput (1 node)", ticks_per_sec, (elapsed_s / TICKS) * 1e6, TICKS, elapsed_s);
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    std::printf("╔══════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n");
    std::printf("║  HORUS C++ FFI Benchmark Suite                                                                         ║\n");
    std::printf("║  All measurements from the C++ side (what users experience)                  All times in nanoseconds   ║\n");
    std::printf("╠══════════════════════════════════════════════════════════════════════════════════════════════════════════╣\n");

    const int N = 10000;

    bench_ffi_call_overhead(N);
    bench_is_running(N);
    bench_scheduler_create_destroy(1000);
    bench_tick_once_empty(N);
    bench_tick_once_1_node(N);
    bench_tick_once_10_nodes(N);
    bench_tick_once_50_nodes(N);
    bench_throughput();

    std::printf("╚══════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n");
    return 0;
}
