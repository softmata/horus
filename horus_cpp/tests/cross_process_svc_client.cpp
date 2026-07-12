// Cross-Process Service Client — sends N JSON requests, counts responses.
// Usage: ./cross_process_svc_client <service_name> <client_id> [num_requests]
// Default N = 100. Client exits 0 if it received >= 99% of responses.

#include <horus/horus_c.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "Usage: %s <svc_name> <client_id> [num_requests]\n", argv[0]);
        return 1;
    }
    const char* svc = argv[1];
    const char* client_id = argv[2];
    int num_requests = argc > 3 ? std::atoi(argv[3]) : 100;

    char req_topic[256], res_topic[256];
    std::snprintf(req_topic, sizeof(req_topic), "%s.request", svc);
    std::snprintf(res_topic, sizeof(res_topic), "%s.response.%s", svc, client_id);

    HorusPublisher* pub_ = horus_publisher_json_wire_new(req_topic);
    HorusSubscriber* sub = horus_subscriber_json_wire_new(res_topic);
    if (!pub_ || !sub) { std::fprintf(stderr, "Failed to create topics\n"); return 1; }

    int successes = 0;
    int last_sum = 0;
    for (int i = 0; i < num_requests; i++) {
        // Each request: {a: i, b: i+1} → expected sum 2i+1
        char req_json[64];
        int a = i;
        int b = i + 1;
        int req_len = std::snprintf(req_json, sizeof(req_json), "{\"a\":%d,\"b\":%d}", a, b);
        HorusJsonWireMsg req = {};
        std::memcpy(req.data, req_json, req_len);
        req.data_len = req_len;
        req.msg_id = static_cast<uint64_t>(i + 1);
        req.msg_type = 0;
        horus_publisher_json_wire_send(pub_, &req);

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        bool got = false;
        while (std::chrono::steady_clock::now() < deadline) {
            HorusJsonWireMsg resp;
            if (horus_subscriber_json_wire_recv(sub, &resp)) {
                if (resp.msg_id == req.msg_id) {
                    char json[4000] = {};
                    std::memcpy(json, resp.data, resp.data_len);
                    if (auto* p = std::strstr(json, "\"sum\":")) last_sum = std::atoi(p + 6);
                    got = true;
                    successes++;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
        if (!got) std::fprintf(stderr, "[client] miss at i=%d msg_id=%d\n", i, i + 1);
    }

    std::printf("RESPONSE:last_sum=%d\n", last_sum);
    std::printf("SUCCESSES:%d/%d\n", successes, num_requests);
    std::fflush(stdout);
    horus_publisher_json_wire_destroy(pub_);
    horus_subscriber_json_wire_destroy(sub);
    int threshold = num_requests * 99 / 100;
    return successes >= threshold ? 0 : 1;
}
