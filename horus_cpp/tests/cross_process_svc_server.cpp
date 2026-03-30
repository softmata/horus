// Cross-Process Service Server — reads JSON requests, computes response, publishes back.
// Usage: ./cross_process_svc_server <service_name> <client_id>
#include <horus/horus_c.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "Usage: %s <svc_name> <client_id>\n", argv[0]); return 1; }
    const char* svc = argv[1];
    const char* client_id = argv[2];

    char req_topic[256], res_topic[256];
    std::snprintf(req_topic, sizeof(req_topic), "%s.request", svc);
    std::snprintf(res_topic, sizeof(res_topic), "%s.response.%s", svc, client_id);

    HorusSubscriber* sub = horus_subscriber_json_wire_new(req_topic);
    HorusPublisher* pub_ = horus_publisher_json_wire_new(res_topic);
    if (!sub || !pub_) { std::fprintf(stderr, "Failed to create topics\n"); return 1; }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    int processed = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        HorusJsonWireMsg req;
        if (horus_subscriber_json_wire_recv(sub, &req)) {
            // Parse request — simple add: extract a and b from JSON
            char json[4000] = {};
            std::memcpy(json, req.data, req.data_len);

            int a = 0, b = 0;
            // Crude JSON parse (production code would use a real parser)
            if (auto* p = std::strstr(json, "\"a\":")) a = atoi(p + 4);
            if (auto* p = std::strstr(json, "\"b\":")) b = atoi(p + 4);

            // Compute response
            char resp_json[256];
            int len = std::snprintf(resp_json, sizeof(resp_json), "{\"sum\":%d}", a + b);

            HorusJsonWireMsg resp = {};
            std::memcpy(resp.data, resp_json, len);
            resp.data_len = len;
            resp.msg_id = req.msg_id;
            resp.msg_type = 1; // response_ok

            horus_publisher_json_wire_send(pub_, &resp);
            std::printf("PROCESSED:%lu:sum=%d\n", req.msg_id, a + b);
            std::fflush(stdout);
            processed++;
            if (processed >= 5) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("TOTAL_PROCESSED:%d\n", processed);
    horus_subscriber_json_wire_destroy(sub);
    horus_publisher_json_wire_destroy(pub_);
    return 0;
}
