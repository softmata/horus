// End-to-end C++ action test: a server (driven by process()) runs a goal on its
// own thread, and a cancel sent over the topic reaches the RUNNING goal.
//
// Build (headless): link against libhorus_cpp.a with -lpthread -ldl.
// Returns 0 on success, non-zero on failure.

#include <horus/action.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <unistd.h>

static std::atomic<bool> g_started{false};
static std::atomic<bool> g_saw_cancel{false};
static std::atomic<bool> g_hit_guard{false};

static uint8_t accept_handler(const uint8_t*, size_t) { return 0; /* accept */ }

// Cooperative long-running goal: exits early iff cancellation is observed.
static void execute_handler(HorusActionGoalHandle* h, const uint8_t*, size_t) {
    horus::ActionGoalHandle goal(h);
    g_started.store(true);
    for (int i = 0; i < 500; ++i) {
        if (goal.is_cancel_requested()) {
            g_saw_cancel.store(true);
            goal.canceled(R"({"canceled":true})");
            return;
        }
        goal.publish_feedback(R"({"progress":0.1})");
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    g_hit_guard.store(true);
    goal.succeed(R"({"canceled":false})");
}

static bool fail(const char* msg) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    return false;
}

static bool test_cancel_running_goal() {
    const std::string name = "cpp_e2e_action." + std::to_string(::getpid());
    horus::ActionServer server(name);
    server.set_accept_handler(accept_handler);
    server.set_execute_handler(execute_handler);
    if (!server.is_ready()) return fail("server not ready");

    horus::ActionClient client(name);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto goal = client.send_goal(R"({"x":1.0})");
    if (!goal) return fail("send_goal returned null");
    uint64_t goal_id = goal.id();
    if (goal_id == 0) return fail("goal id is zero");

    // Drive the server until the goal is actually executing.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!g_started.load()) {
        server.process();
        if (std::chrono::steady_clock::now() > deadline) return fail("goal never started");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Cancel it over the topic while it runs.
    client.cancel(goal_id);

    // Keep driving so process() delivers the cancel.
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!g_saw_cancel.load() && !g_hit_guard.load()) {
        server.process();
        if (std::chrono::steady_clock::now() > deadline) return fail("cancel never observed");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!g_saw_cancel.load()) return fail("running goal did not observe the topic cancel");
    if (g_hit_guard.load()) return fail("goal ran to its guard instead of canceling");

    // Client should be able to read a terminal (Canceled) result.
    horus::GoalStatus status = horus::GoalStatus::Pending;
    std::string result_json;
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool got = false;
    while (std::chrono::steady_clock::now() <= deadline) {
        server.process();
        if (client.poll_result(goal_id, status, result_json)) { got = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!got) return fail("client never received a result");
    if (status != horus::GoalStatus::Canceled) return fail("result status was not Canceled");

    return true;
}

int main() {
    if (!test_cancel_running_goal()) return 1;
    std::printf("OK: C++ action server processes goals and cancels a running goal\n");
    return 0;
}
