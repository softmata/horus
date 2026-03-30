// Cross-language test: C++ publisher
// Publishes CmdVel on "cmd_vel" topic for Python/Rust subscribers
// Usage: ./cross_lang_cpp_pub <count> [topic]

#include <horus/horus_c.h>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    int count = argc > 1 ? std::atoi(argv[1]) : 10;
    const char* topic = argc > 2 ? argv[2] : "cmd_vel";

    HorusPublisher* pub = horus_publisher_cmd_vel_new(topic);
    if (!pub) { std::fprintf(stderr, "Failed to create publisher\n"); return 1; }

    for (int i = 0; i < count; i++) {
        HorusCmdVel msg = {};
        msg.timestamp_ns = (uint64_t)i;
        msg.linear = (float)i * 0.1f;
        msg.angular = (float)i * -0.05f;
        horus_publisher_cmd_vel_send(pub, &msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    horus_publisher_cmd_vel_destroy(pub);
    std::printf("CPP_PUB_DONE:%d\n", count);
    std::fflush(stdout);
    return 0;
}
