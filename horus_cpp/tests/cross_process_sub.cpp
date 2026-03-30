// Cross-Process Subscriber — receives CmdVel messages and prints them.
// Used by Rust integration tests to verify cross-process SHM IPC.
//
// Protocol:
//   1. Reads topic name from argv[1]
//   2. Polls for messages for up to 2 seconds
//   3. Prints "RECEIVED:timestamp_ns:linear" for each message
//   4. Exits

#include <horus/horus_c.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <topic_name>\n", argv[0]);
        return 1;
    }
    const char* topic = argv[1];

    HorusSubscriber* sub = horus_subscriber_cmd_vel_new(topic);
    if (!sub) {
        std::fprintf(stderr, "Failed to create subscriber for '%s'\n", topic);
        return 1;
    }

    int received = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (std::chrono::steady_clock::now() < deadline) {
        HorusCmdVel msg;
        if (horus_subscriber_cmd_vel_recv(sub, &msg)) {
            std::printf("RECEIVED:%lu:%.2f\n", msg.timestamp_ns, msg.linear);
            std::fflush(stdout);
            received++;
            if (received >= 10) break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::printf("TOTAL:%d\n", received);
    std::fflush(stdout);
    horus_subscriber_cmd_vel_destroy(sub);
    return 0;
}
