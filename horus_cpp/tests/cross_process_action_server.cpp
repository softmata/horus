// Cross-Process Action Server — accepts goal, sends 3 feedback, sends result.
// Usage: ./cross_process_action_server <action_name>
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

    HorusSubscriber* goal_sub = horus_subscriber_json_wire_new(goal_t);
    HorusPublisher* fb_pub = horus_publisher_json_wire_new(fb_t);
    HorusPublisher* res_pub = horus_publisher_json_wire_new(res_t);
    if (!goal_sub || !fb_pub || !res_pub) { std::fprintf(stderr, "Topic init failed\n"); return 1; }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        HorusJsonWireMsg goal;
        if (horus_subscriber_json_wire_recv(goal_sub, &goal)) {
            std::printf("GOAL_RECEIVED:%lu\n", goal.msg_id);

            // Send 3 feedback messages
            for (int i = 1; i <= 3; i++) {
                HorusJsonWireMsg fb = {};
                char json[128];
                int len = std::snprintf(json, 128, "{\"progress\":%.1f}", i / 3.0);
                std::memcpy(fb.data, json, len);
                fb.data_len = len;
                fb.msg_id = goal.msg_id;
                fb.msg_type = 3; // feedback
                horus_publisher_json_wire_send(fb_pub, &fb);
                std::printf("FEEDBACK:%d\n", i);
            }

            // Send result
            HorusJsonWireMsg res = {};
            const char* rj = "{\"success\":true}";
            std::memcpy(res.data, rj, std::strlen(rj));
            res.data_len = std::strlen(rj);
            res.msg_id = goal.msg_id;
            res.msg_type = 5; // result
            horus_publisher_json_wire_send(res_pub, &res);
            std::printf("RESULT_SENT\n");
            std::fflush(stdout);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    horus_subscriber_json_wire_destroy(goal_sub);
    horus_publisher_json_wire_destroy(fb_pub);
    horus_publisher_json_wire_destroy(res_pub);
    return 0;
}
