// Long-running C++ node for CLI introspection testing.
// Publishes CmdVel at 10Hz for the specified duration.
// Usage: ./cpp_live_node <duration_seconds> [topic_name]
#include <horus/horus_c.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <chrono>
#include <thread>

static int tick_count = 0;
void node_tick() { tick_count++; }

int main(int argc, char** argv) {
    int duration = argc > 1 ? std::atoi(argv[1]) : 10;
    const char* topic = argc > 2 ? argv[2] : "cpp_live.cmd_vel";

    HorusScheduler* sched = horus_scheduler_new();
    horus_scheduler_name(sched, "cpp_live_sched");

    HorusNodeBuilder* b = horus_node_builder_new("cpp_live_node");
    horus_node_builder_set_tick(b, node_tick);
    horus_node_builder_build(b, sched);

    HorusPublisher* pub_ = horus_publisher_cmd_vel_new(topic);

    std::printf("STARTED:pid=%d:topic=%s:duration=%ds\n", (int)getpid(), topic, duration);
    std::fflush(stdout);

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    int msg_count = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        HorusCmdVel msg = {};
        msg.timestamp_ns = (uint64_t)msg_count;
        msg.linear = 0.5f;
        msg.angular = 0.1f;
        horus_publisher_cmd_vel_send(pub_, &msg);
        horus_scheduler_tick_once(sched);
        msg_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    horus_publisher_cmd_vel_destroy(pub_);
    horus_scheduler_destroy(sched);
    std::printf("STOPPED:msgs=%d:ticks=%d\n", msg_count, tick_count);
    return 0;
}
