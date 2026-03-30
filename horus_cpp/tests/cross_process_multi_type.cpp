// Cross-Process Multi-Type IPC Test
//
// Tests all available message types through cross-process SHM.
// Currently tests CmdVel (Pod) and JsonWireMessage (for arbitrary types).
//
// Usage: ./cross_process_multi_type [pub|sub] <topic_prefix>

#include <horus/horus_c.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

void run_publisher(const char* prefix) {
    char topic[256];

    // Test 1: CmdVel
    std::snprintf(topic, 256, "%s.cmd_vel", prefix);
    HorusPublisher* cmd_pub = horus_publisher_cmd_vel_new(topic);
    for (int i = 0; i < 5; i++) {
        HorusCmdVel msg = {};
        msg.timestamp_ns = (uint64_t)i;
        msg.linear = (float)i * 0.1f;
        msg.angular = (float)i * -0.05f;
        horus_publisher_cmd_vel_send(cmd_pub, &msg);
    }
    horus_publisher_cmd_vel_destroy(cmd_pub);
    std::printf("PUB:cmd_vel:5\n");

    // Test 2: JsonWireMessage (simulating Twist)
    std::snprintf(topic, 256, "%s.twist", prefix);
    HorusPublisher* twist_pub = horus_publisher_json_wire_new(topic);
    for (int i = 0; i < 5; i++) {
        HorusJsonWireMsg msg = {};
        char json[256];
        int len = std::snprintf(json, 256,
            "{\"type\":\"twist\",\"linear\":[%d.0,0,0],\"angular\":[0,0,%d.0]}",
            i, i * 2);
        std::memcpy(msg.data, json, len);
        msg.data_len = len;
        msg.msg_id = (uint64_t)i;
        msg.msg_type = 0;
        horus_publisher_json_wire_send(twist_pub, &msg);
    }
    horus_publisher_json_wire_destroy(twist_pub);
    std::printf("PUB:twist_json:5\n");

    // Test 3: JsonWireMessage (simulating Imu)
    std::snprintf(topic, 256, "%s.imu", prefix);
    HorusPublisher* imu_pub = horus_publisher_json_wire_new(topic);
    for (int i = 0; i < 5; i++) {
        HorusJsonWireMsg msg = {};
        char json[256];
        int len = std::snprintf(json, 256,
            "{\"type\":\"imu\",\"accel_z\":%.2f}", 9.81 + i * 0.01);
        std::memcpy(msg.data, json, len);
        msg.data_len = len;
        msg.msg_id = (uint64_t)(100 + i);
        msg.msg_type = 0;
        horus_publisher_json_wire_send(imu_pub, &msg);
    }
    horus_publisher_json_wire_destroy(imu_pub);
    std::printf("PUB:imu_json:5\n");

    std::fflush(stdout);
}

void run_subscriber(const char* prefix) {
    char topic[256];
    int total = 0;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);

    // CmdVel
    std::snprintf(topic, 256, "%s.cmd_vel", prefix);
    HorusSubscriber* cmd_sub = horus_subscriber_cmd_vel_new(topic);
    int cmd_count = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        HorusCmdVel msg;
        if (horus_subscriber_cmd_vel_recv(cmd_sub, &msg)) {
            cmd_count++;
        } else if (cmd_count > 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("SUB:cmd_vel:%d\n", cmd_count);
    total += cmd_count;
    horus_subscriber_cmd_vel_destroy(cmd_sub);

    // Twist JSON
    std::snprintf(topic, 256, "%s.twist", prefix);
    HorusSubscriber* twist_sub = horus_subscriber_json_wire_new(topic);
    int twist_count = 0;
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        HorusJsonWireMsg msg;
        if (horus_subscriber_json_wire_recv(twist_sub, &msg)) {
            twist_count++;
        } else if (twist_count > 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("SUB:twist_json:%d\n", twist_count);
    total += twist_count;
    horus_subscriber_json_wire_destroy(twist_sub);

    // Imu JSON
    std::snprintf(topic, 256, "%s.imu", prefix);
    HorusSubscriber* imu_sub = horus_subscriber_json_wire_new(topic);
    int imu_count = 0;
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        HorusJsonWireMsg msg;
        if (horus_subscriber_json_wire_recv(imu_sub, &msg)) {
            imu_count++;
        } else if (imu_count > 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("SUB:imu_json:%d\n", imu_count);
    total += imu_count;
    horus_subscriber_json_wire_destroy(imu_sub);

    std::printf("TOTAL:%d\n", total);
    std::fflush(stdout);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "Usage: %s [pub|sub] <topic_prefix>\n", argv[0]);
        return 1;
    }
    if (std::strcmp(argv[1], "pub") == 0) {
        run_publisher(argv[2]);
    } else {
        run_subscriber(argv[2]);
    }
    return 0;
}
