// Cross-Process IPC Test — concurrent publisher and subscriber
//
// Mode "pub": creates topic, publishes 100 messages over 2 seconds, exits
// Mode "sub": creates topic, polls for 3 seconds, prints count
//
// Usage: ./cross_process_ipc_test pub <topic> | ./cross_process_ipc_test sub <topic>

#include <horus/horus_c.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "Usage: %s [pub|sub] <topic>\n", argv[0]); return 1; }
    bool is_pub = std::strcmp(argv[1], "pub") == 0;
    const char* topic = argv[2];

    if (is_pub) {
        HorusPublisher* pub_ = horus_publisher_cmd_vel_new(topic);
        if (!pub_) { std::fprintf(stderr, "pub create failed\n"); return 1; }
        for (int i = 0; i < 100; i++) {
            HorusCmdVel msg = {};
            msg.timestamp_ns = (uint64_t)i;
            msg.linear = (float)i * 0.01f;
            msg.angular = (float)i * -0.005f;
            horus_publisher_cmd_vel_send(pub_, &msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        horus_publisher_cmd_vel_destroy(pub_);
        std::printf("PUB_DONE:100\n");
    } else {
        HorusSubscriber* sub = horus_subscriber_cmd_vel_new(topic);
        if (!sub) { std::fprintf(stderr, "sub create failed\n"); return 1; }
        int count = 0;
        float last_linear = -1;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            HorusCmdVel msg;
            if (horus_subscriber_cmd_vel_recv(sub, &msg)) {
                count++;
                last_linear = msg.linear;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        horus_subscriber_cmd_vel_destroy(sub);
        std::printf("SUB_DONE:%d:last_linear=%.3f\n", count, last_linear);
    }
    std::fflush(stdout);
    return 0;
}
