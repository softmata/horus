// Cross-Process Action Client — sends goal, reads feedback + result.
// Usage: ./cross_process_action_client <action_name>
#include <horus/horus_c.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "Usage: %s <action>\n", argv[0]); return 1; }
    char goal_t[256], fb_t[256], res_t[256];
    std::snprintf(goal_t, 256, "%s.goal", argv[1]);
    std::snprintf(fb_t, 256, "%s.feedback", argv[1]);
    std::snprintf(res_t, 256, "%s.result", argv[1]);

    HorusPublisher* goal_pub = horus_publisher_json_wire_new(goal_t);
    HorusSubscriber* fb_sub = horus_subscriber_json_wire_new(fb_t);
    HorusSubscriber* res_sub = horus_subscriber_json_wire_new(res_t);
    if (!goal_pub || !fb_sub || !res_sub) { std::fprintf(stderr, "Topic init failed\n"); return 1; }

    // Send goal
    HorusJsonWireMsg goal = {};
    const char* gj = "{\"target_x\":5.0}";
    std::memcpy(goal.data, gj, std::strlen(gj));
    goal.data_len = std::strlen(gj);
    goal.msg_id = 99;
    goal.msg_type = 4; // goal
    horus_publisher_json_wire_send(goal_pub, &goal);
    std::printf("GOAL_SENT:99\n");

    // Read feedback + result
    int fb_count = 0;
    bool got_result = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline && !got_result) {
        HorusJsonWireMsg msg;
        if (horus_subscriber_json_wire_recv(fb_sub, &msg)) {
            char json[4000] = {};
            std::memcpy(json, msg.data, msg.data_len);
            std::printf("FEEDBACK_RECV:%s\n", json);
            fb_count++;
        }
        if (horus_subscriber_json_wire_recv(res_sub, &msg)) {
            char json[4000] = {};
            std::memcpy(json, msg.data, msg.data_len);
            std::printf("RESULT_RECV:%s\n", json);
            got_result = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("FEEDBACK_COUNT:%d\nGOT_RESULT:%d\n", fb_count, got_result ? 1 : 0);
    std::fflush(stdout);

    horus_publisher_json_wire_destroy(goal_pub);
    horus_subscriber_json_wire_destroy(fb_sub);
    horus_subscriber_json_wire_destroy(res_sub);
    return got_result ? 0 : 1;
}
