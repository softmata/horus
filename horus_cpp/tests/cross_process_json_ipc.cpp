// Cross-Process JSON IPC — tests arbitrary message types via JsonWireMessage
// Usage: ./cross_process_json_ipc [pub|sub] <topic>
#include <horus/horus_c.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 3) return 1;
    bool is_pub = std::strcmp(argv[1], "pub") == 0;
    const char* topic = argv[2];

    if (is_pub) {
        HorusPublisher* pub_ = horus_publisher_json_wire_new(topic);
        const char* types[] = {"twist", "imu", "odometry", "laser_scan", "joint_state"};
        for (int i = 0; i < 50; i++) {
            HorusJsonWireMsg msg = {};
            char json[256];
            int len = std::snprintf(json, 256, "{\"type\":\"%s\",\"seq\":%d,\"value\":%.2f}",
                types[i % 5], i, (float)i * 0.1f);
            std::memcpy(msg.data, json, len);
            msg.data_len = len;
            msg.msg_id = (uint64_t)i;
            msg.msg_type = 0;
            horus_publisher_json_wire_send(pub_, &msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        horus_publisher_json_wire_destroy(pub_);
        std::printf("PUB_DONE:50\n");
    } else {
        HorusSubscriber* sub = horus_subscriber_json_wire_new(topic);
        int count = 0;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            HorusJsonWireMsg msg;
            if (horus_subscriber_json_wire_recv(sub, &msg)) {
                count++;
                if (count <= 3) {
                    char json[4000] = {};
                    std::memcpy(json, msg.data, msg.data_len);
                    std::printf("RECV:%s\n", json);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        horus_subscriber_json_wire_destroy(sub);
        std::printf("SUB_DONE:%d\n", count);
    }
    std::fflush(stdout);
    return 0;
}
