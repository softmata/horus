// Regression test: nodes registered from two different translation units must
// each get their own callback slot. See cpp_multi_tu_node.cpp for the second TU
// and the history — every other C++ test in this suite is single-TU, which is
// why nothing here pinned the header's internal-linkage callback registry.

#include <horus/horus.hpp>
#include <gtest/gtest.h>

#include <atomic>

using namespace horus::literals;

// Defined in cpp_multi_tu_node.cpp.
extern std::atomic<int> g_tu2_ticks;
void register_second_tu_node(horus::Scheduler& sched);

static std::atomic<int> g_tu1_ticks{0};

TEST(MultiTranslationUnit, BothNodesRunTheirOwnTickBody) {
    horus::Scheduler sched;
    sched.tick_rate(100_hz).name("multi_tu");

    sched.add("tu1_node")
        .tick([] { g_tu1_ticks.fetch_add(1, std::memory_order_relaxed); })
        .build();

    register_second_tu_node(sched);

    for (int i = 0; i < 10; ++i) sched.tick_once();

    // With per-TU slot counters both nodes registered trampoline 0, so one of
    // these two counters stayed at zero while the other counted double.
    EXPECT_GT(g_tu1_ticks.load(std::memory_order_relaxed), 0)
        << "node from TU 1 never ticked";
    EXPECT_GT(g_tu2_ticks.load(std::memory_order_relaxed), 0)
        << "node from TU 2 never ticked";
    EXPECT_EQ(g_tu1_ticks.load(std::memory_order_relaxed),
              g_tu2_ticks.load(std::memory_order_relaxed))
        << "one tick body ran in place of the other";
}
