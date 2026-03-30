// Cross-Process Publisher — sends CmdVel messages then exits.
// Used by Rust integration tests to verify cross-process SHM IPC.
//
// Protocol:
//   1. Reads topic name from argv[1]
//   2. Publishes 10 messages with timestamp_ns = 0..9, linear = i*0.1
//   3. Prints "PUBLISHED:N" to stdout for each message
//   4. Exits

#include <horus/horus_c.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <topic_name>\n", argv[0]);
        return 1;
    }
    const char* topic = argv[1];

    HorusPublisher* pub_ = horus_publisher_cmd_vel_new(topic);
    if (!pub_) {
        std::fprintf(stderr, "Failed to create publisher for '%s'\n", topic);
        return 1;
    }

    for (int i = 0; i < 10; i++) {
        HorusCmdVel msg;
        msg.timestamp_ns = (uint64_t)i;
        msg.linear = (float)i * 0.1f;
        msg.angular = 0.0f;
        horus_publisher_cmd_vel_send(pub_, &msg);
        std::printf("PUBLISHED:%d\n", i);
        std::fflush(stdout);
    }

    horus_publisher_cmd_vel_destroy(pub_);
    return 0;
}
