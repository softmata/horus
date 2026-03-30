// Cross-Process Service Client — sends JSON request, waits for response.
// Usage: ./cross_process_svc_client <service_name> <client_id>
#include <horus/horus_c.h>
#include <cstdio>
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

    HorusPublisher* pub_ = horus_publisher_json_wire_new(req_topic);
    HorusSubscriber* sub = horus_subscriber_json_wire_new(res_topic);
    if (!pub_ || !sub) { std::fprintf(stderr, "Failed to create topics\n"); return 1; }

    // Send request: {a:3, b:4}
    const char* req_json = "{\"a\":3,\"b\":4}";
    HorusJsonWireMsg req = {};
    std::memcpy(req.data, req_json, std::strlen(req_json));
    req.data_len = std::strlen(req_json);
    req.msg_id = 42;
    req.msg_type = 0; // request
    horus_publisher_json_wire_send(pub_, &req);
    std::printf("SENT:42:{a:3,b:4}\n");
    std::fflush(stdout);

    // Wait for response
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        HorusJsonWireMsg resp;
        if (horus_subscriber_json_wire_recv(sub, &resp)) {
            if (resp.msg_id == 42) {
                char json[4000] = {};
                std::memcpy(json, resp.data, resp.data_len);
                std::printf("RESPONSE:%s\n", json);
                std::fflush(stdout);
                horus_publisher_json_wire_destroy(pub_);
                horus_subscriber_json_wire_destroy(sub);
                return 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("TIMEOUT\n");
    horus_publisher_json_wire_destroy(pub_);
    horus_subscriber_json_wire_destroy(sub);
    return 1;
}
