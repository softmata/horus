// Second translation unit for the multi-TU callback-registry regression test.
//
// Registering a node from a *different* .cpp file than the test body is the
// whole point of this test: the per-node callback registry in
// <horus/impl/scheduler_impl.hpp> used to be declared `static` at namespace
// scope in a header, so every TU got a private copy of the array and of the
// slot counter. Both TUs then claimed slot 0 and handed Rust the same
// `tick_trampoline_0`, of which the linker keeps exactly one body — so one
// node ran the other's tick body, or silently never ticked at all.

#include <horus/horus.hpp>

#include <atomic>

// Defined here, observed by the test TU.
std::atomic<int> g_tu2_ticks{0};

void register_second_tu_node(horus::Scheduler& sched) {
    sched.add("tu2_node")
        .tick([] { g_tu2_ticks.fetch_add(1, std::memory_order_relaxed); })
        .build();
}
